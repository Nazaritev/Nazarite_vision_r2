#!/usr/bin/env python3
import sys
from pathlib import Path

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

import rclpy

from stm32_driver.r2_mode1 import Point3d
from stm32_driver.r1_mode1_challenge_blue import R1Mode1ChallengeBlue


class R1Mode1ChallengeRed(R1Mode1ChallengeBlue):
    CHALLENGE_BARCODE_POINTS = (
        Point3d(-0.80, 0.21, 0.0, 0.0),
        Point3d(-0.80, 0.39, 0.0, 0.0),
        Point3d(-0.80, 0.59, 0.0, 0.0),
    )
    CHALLENGE_RELATIVE_OFFSETS = {
        1: -0.23,
        2: -0.02,
    }

    def __init__(self, node_name: str = "r1_mode1_challenge_red"):
        super().__init__(node_name=node_name)

    def apply_blue_mirror_config(self):
        self.get_logger().info(
            "红方挑战模式已启用：不执行蓝方镜像；"
            "challenge 点位按 x 轴对称后的红方坐标发布"
        )


def main():
    rclpy.init(args=sys.argv)
    node = None
    try:
        node = R1Mode1ChallengeRed()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
