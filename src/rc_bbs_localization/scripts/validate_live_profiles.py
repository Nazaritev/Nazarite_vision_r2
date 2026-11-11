#!/usr/bin/python3
"""Observe YAML-defined hardware trials and restore production files afterward."""
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone
import yaml
from ament_index_python.packages import get_package_share_directory


def main():
    share = Path(get_package_share_directory('rc_bringup'))
    config_dir = share / 'config/reality'
    cfg = yaml.safe_load((config_dir / 'rc_localization_overlap_validation.yaml').read_text())
    parameter_file = Path(cfg['localization_config'])
    validation_file = config_dir / 'rc_localization_validation.yaml'
    parameter_text = parameter_file.read_text()
    validation_text = validation_file.read_text()
    script = Path(__file__).resolve().with_name('validate_live_localization.py')
    capture_root = Path(cfg['live_capture_root']) / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    for profile in cfg['live_profiles']:
        out = Path(cfg['output_dir']) / 'live_profiles' / profile['name']
        out.mkdir(parents=True, exist_ok=True)
        capture = capture_root / profile['name']
        if capture.exists():
            raise FileExistsError(f'Remove or rename the old capture before rerunning: {capture}')
        try:
            params = yaml.safe_load(parameter_text)
            params['bbs_localizer']['ros__parameters'].update(profile['overrides'])
            actual = yaml.safe_dump(params, sort_keys=False)
            (out / 'parameters.yaml').write_text(actual)
            parameter_file.write_text(actual)
            validation = yaml.safe_load(validation_text)
            validation.update(output_dir=str(out), live_duration_sec=cfg['live_duration_sec'],
                              live_capture_bag_dir=str(capture))
            validation_file.write_text(yaml.safe_dump(validation, sort_keys=False))
            with (out / 'observer.log').open('w') as log:
                subprocess.run([sys.executable, str(script)], check=True, stdout=log,
                               stderr=subprocess.STDOUT, env=dict(os.environ, ROS_LOG_DIR=str(capture.parent / 'logs')))
        finally:
            parameter_file.write_text(parameter_text)
            validation_file.write_text(validation_text)


if __name__ == '__main__':
    main()
