#!/usr/bin/python3
"""Re-run saved fixtures, tuning profiles, ROS recovery tests and summary.

Configuration: rc_bringup/config/reality/rc_localization_validation.yaml.
Extracting new bags is a separate tool so a missing /tmp bag cannot overwrite
saved real-scan fixtures. No parameter values are taken from the terminal.
"""
import os
from pathlib import Path
import subprocess
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import yaml


def main():
    share=Path(get_package_share_directory('rc_bringup'))
    config=share/'config/reality/rc_localization_validation.yaml'
    prefix=Path(get_package_prefix('rc_bbs_localization'))/'lib/rc_bbs_localization'
    settings=yaml.safe_load(config.read_text())
    env=dict(os.environ,OMP_WAIT_POLICY='PASSIVE')
    subprocess.run([str(prefix/'bbs_benchmark'),str(config)],check=True,env=env)
    subprocess.run([str(prefix/'validate_ros_pipeline.py')],check=True,env=env)
    subprocess.run([str(prefix/'summarize_validation.py')],check=True,env=env)
    print(f'Validation data: {settings["output_dir"]}')


if __name__=='__main__':
    main()
