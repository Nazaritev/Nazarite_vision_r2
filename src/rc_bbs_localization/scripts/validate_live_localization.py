#!/usr/bin/python3
"""Observe the new production launch on real hardware without publishing commands or seeds."""
import os
from pathlib import Path
import json
import signal
import subprocess
import time
from collections import Counter
import yaml
from ament_index_python.packages import get_package_share_directory


def main():
    share=Path(get_package_share_directory('rc_bringup'))
    cfg=yaml.safe_load((share/'config/reality/rc_localization_validation.yaml').read_text())
    settings=yaml.safe_load((share/'config/reality/rc_localization_launch.yaml').read_text())
    if settings['input_mode']!='live':raise ValueError('Live validation requires input_mode: live in launch YAML')
    os.environ['ROS_DOMAIN_ID']=str(settings['ros_domain_id'])
    os.environ['OMP_WAIT_POLICY']='PASSIVE'
    if settings['dds_profile']=='local':
        os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
        os.environ['CYCLONEDDS_URI']=(share/'config/reality/rc_localization_dds.xml').as_uri()
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import PoseStamped
    out=Path(cfg['output_dir']);out.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('bbs_live_validation_observer')
    rows=[];poses=[];scan_count=0
    def status(message):
        if not message.status:return
        row={v.key:v.value for v in message.status[0].values}
        row['stamp_ns']=message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        rows.append(row)
    def cloud(message):
        nonlocal scan_count
        scan_count+=1
    def pose(message):
        poses.append({'stamp_ns':message.header.stamp.sec*1000000000+message.header.stamp.nanosec,
                      'xyz':[message.pose.position.x,message.pose.position.y,message.pose.position.z]})
    node.create_subscription(DiagnosticArray,'/bbs_localization/status',status,10)
    from rclpy.qos import qos_profile_sensor_data
    node.create_subscription(PointCloud2,'/cloud_registered_body',cloud,qos_profile_sensor_data)
    node.create_subscription(PoseStamped,'/bbs_localization/sensor_pose',pose,10)
    log=(out/'live_launch.log').open('w')
    recorder=None
    capture=cfg.get('live_capture_bag_dir')
    if capture:
        bag_dir=Path(capture)
        bag_dir.parent.mkdir(parents=True,exist_ok=True)
        if bag_dir.exists():raise FileExistsError(f'Refusing to overwrite capture: {bag_dir}')
        recorder=subprocess.Popen([
            'ros2','bag','record','-o',str(bag_dir),
            '/cloud_registered_body','/Odometry','/livox/imu','/tf','/tf_static',
        ],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    process=subprocess.Popen(['ros2','launch','rc_bringup','rc_localization.launch.py'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        start=time.monotonic()
        while time.monotonic()-start<cfg.get('live_duration_sec',18.):
            rclpy.spin_once(node,timeout_sec=.02)
            if process.poll() is not None:break
    finally:
        for child in (process,recorder):
            if child is None:continue
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=6)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        node.destroy_node();rclpy.shutdown();log.close()
    counts=Counter((r.get('state'),r.get('reason')) for r in rows)
    result={'has_pose_ground_truth':False,'capture_bag_dir':capture,'body_scan_messages':scan_count,'verified_pose_messages':len(poses),
            'counts':{str(k):v for k,v in counts.items()},'last_status':rows[-1] if rows else None,
            'status_samples':rows,'verified_poses':poses}
    (out/'live_validation.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('status_samples','verified_poses')},indent=2))


if __name__=='__main__':main()
