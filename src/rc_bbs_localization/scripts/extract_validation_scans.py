#!/usr/bin/python3
"""Extract real body scans and synchronized, stable IMU priors without replaying motion commands."""
from pathlib import Path
import sqlite3
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from ament_index_python.packages import get_package_share_directory


def quaternion_rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def xyz_points(message):
    fields = {f.name: f for f in message.fields}
    if any(fields[n].datatype != 7 for n in ('x', 'y', 'z')):
        raise ValueError('Expected float32 xyz')
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype({'names': ['x','y','z'], 'formats': [endian+'f4']*3,
                      'offsets': [fields[n].offset for n in ('x','y','z')],
                      'itemsize': message.point_step})
    structured = np.ndarray((message.height, message.width), dtype=dtype,
                            buffer=message.data, strides=(message.row_step, message.point_step))
    return np.column_stack([structured[n].ravel() for n in ('x','y','z')]).astype('<f4')


def pose_matrix(p):
    result=np.eye(4)
    result[:3,:3]=quaternion_rotation(p.orientation)
    result[:3,3]=[p.position.x,p.position.y,p.position.z]
    return result


def extract(config_path):
    cfg = yaml.safe_load(Path(config_path).read_text())
    parameters = yaml.safe_load(Path(cfg['localization_config']).read_text())['bbs_localizer']['ros__parameters']
    window = float(cfg.get('bag_accumulation_sec', parameters['accumulation_sec']))
    max_frames = int(cfg.get('bag_accumulation_max_frames', parameters['accumulation_max_frames']))
    trusted_bags = set(cfg.get('accumulation_bags', []))
    out = Path(cfg['output_dir']) / 'bag_scans'
    out.mkdir(parents=True, exist_ok=True)
    manifest = {'cases': []}
    for bag_index, bag in enumerate(cfg['bags']):
        if not Path(bag).is_file():
            continue
        connection = sqlite3.connect(f'file:{bag}?mode=ro', uri=True)
        topics = dict(connection.execute('select name,id from topics'))
        if '/cloud_registered_body' not in topics:
            continue
        rows = connection.execute(
            'select timestamp,data from messages where topic_id=? order by timestamp',
            (topics['/cloud_registered_body'],)).fetchall()
        imu_rows = connection.execute(
            'select data from messages where topic_id=? order by timestamp',
            (topics['/livox/imu'],)).fetchall()
        imus = [deserialize_message(data, Imu) for (data,) in imu_rows]
        for fraction in cfg['bag_scan_fractions']:
            if not rows:
                continue
            index = min(len(rows)-1, int(len(rows)*fraction))
            message = deserialize_message(rows[index][1], PointCloud2)
            stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
            samples = [m for m in imus if 0 <= stamp-(m.header.stamp.sec+m.header.stamp.nanosec*1e-9) <= .2]
            if len(samples) < 10:
                continue
            acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m in samples])
            gyro = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in samples])
            if np.max(np.linalg.norm(gyro, axis=1)) > .15 or np.max(acc.std(axis=0)) > .25:
                continue
            # Historical driver may name IMU messages livox_frame. Confirm static
            # source-to-IMU rotation from the bag rather than assuming frame identity.
            imu_frame = samples[0].header.frame_id
            rotation = np.eye(3)
            if imu_frame != message.header.frame_id:
                found = False
                for (data,) in connection.execute('select data from messages where topic_id=?', (topics['/tf_static'],)):
                    for t in deserialize_message(data, TFMessage).transforms:
                        if t.header.frame_id == message.header.frame_id and t.child_frame_id == imu_frame:
                            rotation = quaternion_rotation(t.transform.rotation)
                            found = True
                if not found:
                    continue
            xyz=xyz_points(message)
            accumulated=1
            # Accumulation is restricted to the newly captured bag whose odometry
            # and mounting conventions were inspected in the current source.
            if bag in trusted_bags and '/Odometry' in topics:
                extrinsic=None
                for (data,) in connection.execute('select data from messages where topic_id=?', (topics['/tf_static'],)):
                    for t in deserialize_message(data, TFMessage).transforms:
                        if t.header.frame_id=='base_link' and t.child_frame_id=='imu_link':
                            extrinsic=np.eye(4)
                            extrinsic[:3,:3]=quaternion_rotation(t.transform.rotation)
                            extrinsic[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
                odoms=[]
                for (data,) in connection.execute('select data from messages where topic_id=? order by timestamp', (topics['/Odometry'],)):
                    o=deserialize_message(data,Odometry)
                    if o.header.frame_id!=parameters['odom_frame'] or o.child_frame_id!=parameters['base_frame']:
                        continue
                    odoms.append((o.header.stamp.sec+o.header.stamp.nanosec*1e-9,pose_matrix(o.pose.pose)))
                if extrinsic is not None and odoms:
                    times=np.array([t for t,_ in odoms])
                    current=odoms[int(np.argmin(abs(times-stamp)))][1]@extrinsic
                    clouds=[]
                    for k in range(max(0,index-max_frames+1),index+1):
                        old=deserialize_message(rows[k][1],PointCloud2)
                        old_stamp=old.header.stamp.sec+old.header.stamp.nanosec*1e-9
                        if stamp-old_stamp>window:continue
                        j=int(np.argmin(abs(times-old_stamp)))
                        if abs(times[j]-old_stamp)>.03:continue
                        relative=np.linalg.inv(current)@odoms[j][1]@extrinsic
                        points=xyz_points(old)
                        clouds.append(points@relative[:3,:3].T+relative[:3,3])
                    if clouds:
                        xyz=np.concatenate(clouds).astype('<f4');accumulated=len(clouds)
            case = f'bag_{bag_index}_{index}'
            pcd = out / f'{case}.pcd'
            header = f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n'
            with pcd.open('wb') as stream:
                stream.write(header.encode())
                stream.write(xyz.tobytes())
            manifest['cases'].append({'id': case, 'pcd': str(pcd), 'bag': bag,
                                      'scan_frame': message.header.frame_id,
                                      'stamp': stamp, 'has_ground_truth': False, 'accumulated_scan_count': accumulated,
                                      'acceleration_sensor': (rotation @ acc.mean(axis=0)).tolist()})
        connection.close()
    (out / 'manifest.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False))
    print(f'Extracted {len(manifest["cases"])} real scans; ground truth unavailable.')


if __name__ == '__main__':
    extract(Path(get_package_share_directory('rc_bringup')) / 'config/reality/rc_localization_validation.yaml')
