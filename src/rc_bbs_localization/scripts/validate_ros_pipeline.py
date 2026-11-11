#!/usr/bin/python3
"""Runs the actual node on reproducible synthetic scans; never commands a robot.

All fixture selection and timing live in rc_localization_validation.yaml.
Ground truth is used only by this test publisher/scorer and never by bbs_localizer.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '91'
os.environ['OMP_WAIT_POLICY'] = 'PASSIVE'
from pathlib import Path
import csv
import json
import signal
import subprocess
import tempfile
import shutil
import time

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = (Path(get_package_share_directory('rc_bringup')) / 'config/reality/rc_localization_dds.xml').as_uri()


def rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def read_xyz(path):
    with Path(path).open('rb') as stream:
        lines=[]
        while True:
            line=stream.readline().decode().strip()
            lines.append(line)
            if line.startswith('DATA '):
                break
        header=dict(line.split(' ',1) for line in lines if ' ' in line and not line.startswith('#'))
        if header['DATA'] != 'binary' or header['FIELDS'] != 'x y z':
            raise ValueError('Fixture must be binary xyz PCD')
        return np.frombuffer(stream.read(),dtype='<f4').reshape(-1,3).copy()


class Fixture(Node):
    def __init__(self, case, config):
        super().__init__('bbs_validation_fixture')
        self.case=case
        self.xyz=read_xyz(case['pcd'])
        self.rng=np.random.default_rng(config['random_seed']+991)
        self.frame_noise=config.get('ros_frame_noise_std_m',.005)
        self.frame_drop=config.get('ros_frame_drop_fraction',.10)
        self.imu_rate=float(config.get('ros_imu_rate_hz',200.))
        self.scan_rate=float(config.get('ros_scan_rate_hz',40.))
        ratio=self.imu_rate/self.scan_rate
        if self.scan_rate<=0 or self.imu_rate<=0 or abs(ratio-round(ratio))>1e-6:
            raise ValueError('Fixture IMU rate must be a positive integer multiple of scan rate')
        self.scan_every=int(round(ratio))
        self.truth=np.eye(4)
        self.truth[:3,:3]=rotation(case['quaternion_xyzw'])
        self.truth[:3,3]=case['xyz']
        self.extrinsic=np.diag([-1.,1.,-1.,1.])  # same pi around Y as production TF
        self.status={}
        self.events=[]
        self.errors=[]
        self.phase='unknown'
        self.phase_start=time.monotonic()
        self.first_pose={}
        self.counter=0
        self.motion=False
        self.motion_offset=0.
        self.phase_min_stamp=0.
        self.inputs=True
        self.imu_scale=1.
        self.latest_truth=self.truth.copy()
        self.cloud=self.create_publisher(PointCloud2,'/cloud_registered_body',5)
        self.odom=self.create_publisher(Odometry,'/Odometry',50)
        self.imu=self.create_publisher(Imu,'/livox/imu',100)
        self.initial=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.tf=self.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id='imu_link';t.transform.rotation.y=1.
        self.tf.publish(TFMessage(transforms=[t]))
        self.create_subscription(DiagnosticArray,'/bbs_localization/status',self.on_status,10)
        self.create_subscription(PoseStamped,'/bbs_localization/sensor_pose',self.on_pose,10)
        self.create_timer(1./self.imu_rate,self.publish)
        self.reload=self.create_client(Trigger,'/bbs_localization/reload_map')
        self.relocalize=self.create_client(Trigger,'/bbs_localization/relocalize')

    def begin_phase(self,phase):
        self.phase=phase
        self.phase_start=time.monotonic()
        self.phase_min_stamp=self.get_clock().now().nanoseconds*1e-9

    def on_status(self,msg):
        if not msg.status:return
        self.status={p.key:p.value for p in msg.status[0].values}
        state=self.status.get('state')
        event={'phase':self.phase,'elapsed':time.monotonic()-self.phase_start,
               'state':state,'reason':self.status.get('reason'),
               'epoch':self.status.get('epoch'),'active_workers':self.status.get('active_workers')}
        if not self.events or any(event[k]!=self.events[-1].get(k) for k in ('phase','state','reason','epoch')):
            self.events.append(event)

    def on_pose(self,msg):
        if self.phase=='untrusted_gravity':return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp<self.phase_min_stamp:return
        error=float(np.linalg.norm(np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])-self.latest_truth[:3,3]))
        estimate=rotation([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
        angular=float(np.arccos(np.clip((np.trace(estimate.T@self.latest_truth[:3,:3])-1)/2,-1,1)))
        self.errors.append({'phase':self.phase,'translation_error_m':error,'rotation_error_rad':angular})
        self.first_pose.setdefault(self.phase,time.monotonic()-self.phase_start)

    def publish(self):
        self.counter+=1
        if not self.inputs:return
        stamp=self.get_clock().now().to_msg()
        if self.motion:self.motion_offset+=.05/self.imu_rate
        offset=self.motion_offset
        odom=np.eye(4);odom[0,3]=offset
        self.latest_truth=self.truth@self.extrinsic@odom@self.extrinsic
        o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_link'
        o.pose.pose.orientation.w=1.;o.pose.pose.position.x=offset
        self.odom.publish(o)
        m=Imu();m.header.stamp=stamp;m.header.frame_id='imu_link'
        m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=map(float,np.array(self.case['acceleration'])*self.imu_scale)
        self.imu.publish(m)
        if self.counter%self.scan_every:return
        delta=np.linalg.inv(self.latest_truth)@self.truth
        points=self.xyz[self.rng.random(len(self.xyz))>=self.frame_drop]
        xyz=(points@delta[:3,:3].T+delta[:3,3]+self.rng.normal(0.,self.frame_noise,points.shape)).astype('<f4')
        cloud=PointCloud2();cloud.header.stamp=stamp;cloud.header.frame_id='imu_link'
        cloud.height=1;cloud.width=len(xyz);cloud.point_step=12;cloud.row_step=12*len(xyz);cloud.is_dense=False
        cloud.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
        cloud.data=xyz.tobytes();self.cloud.publish(cloud)

    def seed(self,bad,two_d=False):
        m=PoseWithCovarianceStamped();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='map'
        base=self.latest_truth@self.extrinsic
        m.pose.pose.position.x=float(base[0,3]+(20 if bad else 0))
        m.pose.pose.position.y=float(base[1,3]+(20 if bad else 0));m.pose.pose.position.z=float(base[2,3])
        if two_d:
            yaw=float(np.arctan2(base[1,0],base[0,0]))
            m.pose.pose.position.z=0.
            m.pose.pose.orientation.z=float(np.sin(yaw/2));m.pose.pose.orientation.w=float(np.cos(yaw/2))
        elif bad:m.pose.pose.orientation.w=1.
        else:
            # Compose q(sensor) with q(sensor->base)=[0,1,0,0].
            x,y,z,w=self.case['quaternion_xyzw']
            m.pose.pose.orientation.x=-z;m.pose.pose.orientation.y=w
            m.pose.pose.orientation.z=x;m.pose.pose.orientation.w=-y
        self.phase_min_stamp=self.get_clock().now().nanoseconds*1e-9
        self.first_pose.pop(self.phase,None)
        self.initial.publish(m)


def spin_until(node,predicate,timeout):
    start=time.monotonic()
    while time.monotonic()-start<timeout:
        rclpy.spin_once(node,timeout_sec=.01)
        if predicate():return True
    return False


def run(config_path,profile=None):
    cfg=yaml.safe_load(Path(config_path).read_text());base_out=Path(cfg['output_dir'])
    profile=profile or {'name':'robust','overrides':{}}
    out=base_out if profile['name']=='robust' else base_out/'ros_profiles'/profile['name']
    out.mkdir(parents=True,exist_ok=True)
    case=yaml.safe_load((base_out/'fixtures/manifest.yaml').read_text())['cases'][0]
    params=yaml.safe_load(Path(cfg['localization_config']).read_text())
    p=params['bbs_localizer']['ros__parameters']
    p.update(profile.get('overrides',{}))
    temporary_map=tempfile.TemporaryDirectory(prefix='rc_bbs_ros_map_')
    map_copy=Path(temporary_map.name)/'map.pcd'
    shutil.copyfile(cfg['map_file'],map_copy)
    p['map_file']=str(map_copy);p['metrics_file']=str(out/'ros_metrics.csv')
    test_params=out/'ros_test_parameters.yaml';test_params.write_text(yaml.safe_dump(params))
    log=(out/'ros_node.log').open('w')
    process=subprocess.Popen(['ros2','run','rc_bbs_localization','bbs_localizer','--ros-args','--params-file',str(test_params)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Fixture(case,cfg);results={'profile':profile['name'],'scan_rate_hz':cfg.get('ros_scan_rate_hz',40.),'imu_rate_hz':cfg.get('ros_imu_rate_hz',200.)}
    try:
        results['unknown_recovered']=spin_until(node,lambda:'unknown' in node.first_pose,10.)
        if results['unknown_recovered']:
            node.begin_phase('motion');node.motion=True
            spin_until(node,lambda:False,1.5)
            node.motion=False
        node.begin_phase('bad_initialpose');node.seed(True)
        results['bad_initialpose_recovered']=spin_until(node,lambda:'bad_initialpose' in node.first_pose,8.)
        node.begin_phase('trusted_initialpose');node.seed(False)
        results['trusted_initialpose_recovered']=spin_until(node,lambda:'trusted_initialpose' in node.first_pose,5.)
        node.begin_phase('rviz_2d_initialpose');node.seed(False,two_d=True)
        results['rviz_2d_initialpose_recovered']=spin_until(node,lambda:'rviz_2d_initialpose' in node.first_pose,5.)
        node.begin_phase('input_loss');node.inputs=False
        results['input_loss_detected']=spin_until(node,lambda:node.status.get('state')=='LOST',2.)
        node.begin_phase('input_resume');node.inputs=True
        results['input_resume_recovered']=spin_until(node,lambda:'input_resume' in node.first_pose,8.)
        node.begin_phase('untrusted_gravity');node.imu_scale=0.
        spin_until(node,lambda:False,.25)
        if node.relocalize.wait_for_service(timeout_sec=1.):
            node.relocalize.call_async(Trigger.Request())
            results['untrusted_gravity_refused']=spin_until(node,lambda:node.status.get('reason')=='gravity_prior_untrusted_need_stationary_imu_and_odometry' and node.status.get('pose_valid')=='0',2.)
        node.imu_scale=1.
        spin_until(node,lambda:node.status.get('state')=='TRACKING',3.)
        node.begin_phase('seed_during_search')
        if node.relocalize.wait_for_service(timeout_sec=1.):
            node.relocalize.call_async(Trigger.Request())
            results['search_observed_before_seed']=spin_until(node,lambda:node.status.get('state')=='SEARCHING',2.)
            node.seed(False)
            results['seed_during_search_recovered']=spin_until(node,lambda:'seed_during_search' in node.first_pose,5.)
        node.begin_phase('map_reload')
        if node.reload.wait_for_service(timeout_sec=1.):
            node.reload.call_async(Trigger.Request())
            results['map_reload_recovered']=spin_until(node,lambda:'map_reload' in node.first_pose,8.)
        node.begin_phase('map_update_during_search')
        if node.relocalize.wait_for_service(timeout_sec=1.):
            node.relocalize.call_async(Trigger.Request())
            results['search_observed_before_map_update']=spin_until(node,lambda:node.status.get('state')=='SEARCHING',2.)
            old_epoch=int(node.status.get('epoch','0'))
            node.begin_phase('map_update_during_search')
            node.first_pose.pop(node.phase,None)
            replacement=map_copy.with_suffix('.new')
            replacement.write_bytes(b'# validation content-version update; geometry unchanged\n'+map_copy.read_bytes())
            replacement.replace(map_copy)
            results['map_update_during_search_recovered']=spin_until(node,lambda:'map_update_during_search' in node.first_pose,10.)
            results['map_update_epoch_advanced']=int(node.status.get('epoch','0'))>old_epoch
        results['first_pose_seconds']=node.first_pose
        results['errors']=node.errors
        results['events']=node.events
        results['max_translation_error_m']=max((e['translation_error_m'] for e in node.errors),default=None)
        results['max_rotation_error_rad']=max((e['rotation_error_rad'] for e in node.errors),default=None)
        (out/'ros_validation.json').write_text(json.dumps(results,indent=2))
        print(json.dumps({k:v for k,v in results.items() if k not in ('errors','events')},indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close()
        temporary_map.cleanup()
    if not all(value for value in results.values() if isinstance(value,bool)):
        raise SystemExit('At least one ROS pipeline assertion failed; inspect ros_validation.json')


if __name__=='__main__':
    config_path=Path(get_package_share_directory('rc_bringup'))/'config/reality/rc_localization_validation.yaml'
    settings=yaml.safe_load(config_path.read_text())
    for profile in settings.get('ros_profiles',[{'name':'robust','overrides':{}}]):
        run(config_path,profile)
