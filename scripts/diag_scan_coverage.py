#!/usr/bin/env python3

from __future__ import annotations

import math
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanCoverage(Node):
    def __init__(self) -> None:
        super().__init__("go2_diag_scan_coverage")
        self._window = deque(maxlen=20)
        self._sub = self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self._timer = self.create_timer(1.0, self._report)

    def _on_scan(self, msg: LaserScan) -> None:
        finite = [r for r in msg.ranges if math.isfinite(r)]
        frac = (len(finite) / len(msg.ranges)) if msg.ranges else 0.0
        # Rough angular coverage: where we have any finite returns.
        covered = 0.0
        if msg.ranges and msg.angle_increment:
            covered = sum(1 for r in msg.ranges if math.isfinite(r)) * abs(msg.angle_increment)
        self._window.append(
            {
                "frame": msg.header.frame_id,
                "n": len(msg.ranges),
                "frac": frac,
                "angle_min": msg.angle_min,
                "angle_max": msg.angle_max,
                "covered_rad": covered,
                "range_min": msg.range_min,
                "range_max": msg.range_max,
            }
        )

    def _report(self) -> None:
        if not self._window:
            self.get_logger().info("Waiting for /scan ...")
            return
        frac_vals = [x["frac"] for x in self._window]
        covered_vals = [x["covered_rad"] for x in self._window]
        last = self._window[-1]
        self.get_logger().info(
            "frame=%s n=%d frac_finite=%.2f (avg %.2f) covered=%.1f° (avg %.1f°) angle=[%.1f°, %.1f°] range=[%.2f, %.2f]",
            last["frame"],
            last["n"],
            last["frac"],
            statistics.mean(frac_vals),
            math.degrees(last["covered_rad"]),
            math.degrees(statistics.mean(covered_vals)),
            math.degrees(last["angle_min"]),
            math.degrees(last["angle_max"]),
            last["range_min"],
            last["range_max"],
        )


def main() -> None:
    rclpy.init()
    node = ScanCoverage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

