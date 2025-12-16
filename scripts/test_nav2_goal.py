#!/usr/bin/env python3
"""
Fire a NavigateToPose goal against the Nav2 stack to validate end-to-end
planning + control in simulation.

Usage:
  python3 scripts/test_nav2_goal.py --x -8.0 --y 0.5 --yaw 0.0
  python3 scripts/test_nav2_goal.py --x -3 --y 2 --yaw 90 --frame map --wait 120
"""

import argparse
import collections
import math
import random
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def quat_from_yaw(yaw_rad: float):
    """Return (x, y, z, w) quaternion for a yaw in radians."""
    half = yaw_rad * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def parse_args():
    parser = argparse.ArgumentParser(description="Send a Nav2 NavigateToPose goal.")
    parser.add_argument("--x", type=float, default=2.0, help="Target x in frame coordinates (default: 2.0)")
    parser.add_argument("--y", type=float, default=0.5, help="Target y in frame coordinates (default: 0.5)")
    parser.add_argument("--yaw", type=float, default=0.0, help="Target yaw in degrees (default: 0)")
    parser.add_argument("--frame", type=str, default="map", help="Frame for the goal pose (default: map)")
    parser.add_argument("--base-frame", type=str, default="unitree_go2/base_footprint", help="Robot base TF frame (default: unitree_go2/base_footprint)")
    parser.add_argument("--wait", type=float, default=240.0, help="Per-goal timeout seconds (default: 240s)")
    parser.add_argument("--repeat", type=int, default=1, help="Number of goals to send (default: 1)")
    parser.add_argument(
        "--auto",
        action="store_true",
        help="Pick a 'hard' goal automatically from /map free space (recommended).",
    )
    parser.add_argument(
        "--allow-unknown",
        action="store_true",
        help="Allow sampling goal cells from unknown space (-1) as fallback (can be unreachable early in SLAM).",
    )
    parser.add_argument("--min-distance", type=float, default=4.0, help="Min distance from current pose (auto) (default: 4.0m)")
    parser.add_argument("--max-distance", type=float, default=12.0, help="Max distance from current pose (auto) (default: 12.0m)")
    parser.add_argument("--clearance", type=float, default=0.5, help="Free-space clearance radius (auto) (default: 0.5m)")
    parser.add_argument("--seed", type=int, default=0, help="RNG seed for auto goal selection (0 -> time-based)")
    parser.add_argument("--map-wait", type=float, default=120.0, help="Seconds to wait for a non-empty /map (default: 120s)")
    return parser.parse_args()


def wait_for_map(node, timeout_sec: float) -> OccupancyGrid:
    msg = None

    def cb(m):
        nonlocal msg
        msg = m

    sub = node.create_subscription(OccupancyGrid, "/map", cb, 10)
    end = time.time() + timeout_sec
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if msg is not None and msg.info.width > 0 and msg.info.height > 0 and len(msg.data) > 0:
            break
    node.destroy_subscription(sub)
    if msg is None or msg.info.width == 0 or msg.info.height == 0 or len(msg.data) == 0:
        raise TimeoutError("Timed out waiting for /map")
    return msg


def wait_for_scan(node, timeout_sec: float) -> LaserScan:
    msg = None

    def cb(m):
        nonlocal msg
        msg = m

    sub = node.create_subscription(LaserScan, "/scan", cb, 10)
    end = time.time() + timeout_sec
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if msg is not None and msg.ranges:
            break
    node.destroy_subscription(sub)
    if msg is None or not msg.ranges:
        raise TimeoutError("Timed out waiting for /scan")
    return msg


def scan_has_returns(scan: LaserScan) -> bool:
    for r in scan.ranges:
        if math.isfinite(r):
            return True
    return False


def lookup_xy_yaw(node, tf_buffer: Buffer, target_frame: str, source_frame: str, timeout_sec: float):
    end = time.time() + timeout_sec
    while rclpy.ok() and time.time() < end:
        try:
            tf = tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.5))
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return x, y, yaw
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.05)
    raise TimeoutError(f"Timed out waiting for TF {target_frame} -> {source_frame}")


def is_free_with_clearance(grid: OccupancyGrid, mx: int, my: int, radius_cells: int) -> bool:
    w = grid.info.width
    h = grid.info.height
    data = grid.data
    for dy in range(-radius_cells, radius_cells + 1):
        yy = my + dy
        if yy < 0 or yy >= h:
            return False
        row = yy * w
        for dx in range(-radius_cells, radius_cells + 1):
            xx = mx + dx
            if xx < 0 or xx >= w:
                return False
            v = data[row + xx]
            # Only block on clearly occupied cells.
            if v >= 50:
                return False
    return True


def world_to_map(grid: OccupancyGrid, x: float, y: float):
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    mx = int((x - ox) / res)
    my = int((y - oy) / res)
    return mx, my


def map_to_world(grid: OccupancyGrid, mx: int, my: int):
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    gx = ox + (mx + 0.5) * res
    gy = oy + (my + 0.5) * res
    return gx, gy


def is_free_cell(v: int, allow_unknown: bool) -> bool:
    if allow_unknown:
        return v <= 0
    return v == 0


def reachable_cells_from_robot(
    grid: OccupancyGrid,
    robot_x: float,
    robot_y: float,
    max_distance: float,
    allow_unknown: bool,
):
    w = grid.info.width
    h = grid.info.height
    data = grid.data
    res = grid.info.resolution
    max_cells = int(max_distance / res) + 1

    start_mx, start_my = world_to_map(grid, robot_x, robot_y)
    if start_mx < 0 or start_mx >= w or start_my < 0 or start_my >= h:
        return []

    start_idx = start_my * w + start_mx
    if not is_free_cell(data[start_idx], allow_unknown):
        # If the robot cell isn't marked free yet, don't try to flood fill.
        return []

    visited = bytearray(w * h)
    visited[start_idx] = 1
    q = collections.deque([(start_mx, start_my)])
    out = []

    while q:
        mx, my = q.popleft()
        out.append((mx, my))
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx = mx + dx
            ny = my + dy
            if nx < 0 or nx >= w or ny < 0 or ny >= h:
                continue
            if abs(nx - start_mx) > max_cells or abs(ny - start_my) > max_cells:
                continue
            nidx = ny * w + nx
            if visited[nidx]:
                continue
            if not is_free_cell(data[nidx], allow_unknown):
                continue
            visited[nidx] = 1
            q.append((nx, ny))

    return out


def sample_goal_from_map(
    grid: OccupancyGrid,
    robot_x: float,
    robot_y: float,
    min_distance: float,
    max_distance: float,
    clearance: float,
    allow_unknown: bool,
    tries: int = 800,
):
    res = grid.info.resolution
    w = grid.info.width
    h = grid.info.height
    radius_cells = max(1, int(clearance / res))

    # Prefer goals in the robot-reachable connected free-space region (reduces planner failures).
    reachable = reachable_cells_from_robot(grid, robot_x, robot_y, max_distance=max_distance, allow_unknown=allow_unknown)
    if reachable:
        for relax in range(3):
            rc = max(1, int((clearance * (0.75**relax)) / res))
            for _ in range(tries):
                mx, my = random.choice(reachable)
                if not is_free_with_clearance(grid, mx, my, rc):
                    continue
                gx, gy = map_to_world(grid, mx, my)
                d = math.hypot(gx - robot_x, gy - robot_y)
                if d < min_distance or d > max_distance:
                    continue
                return gx, gy

    # Fallback: sample from any free cells in the map.
    free_indices = [i for i, v in enumerate(grid.data) if is_free_cell(v, allow_unknown)]
    if not free_indices:
        raise RuntimeError("No free cells in /map (try exploring more, or pass --allow-unknown).")

    # Try with requested clearance first, then relax clearance a bit if the map is sparse.
    for relax in range(3):
        rc = max(1, int((clearance * (0.75**relax)) / res))
        for _ in range(tries):
            idx = random.choice(free_indices)
            mx = idx % w
            my = idx // w
            if not is_free_with_clearance(grid, mx, my, rc):
                continue
            gx, gy = map_to_world(grid, mx, my)
            d = math.hypot(gx - robot_x, gy - robot_y)
            if d < min_distance or d > max_distance:
                continue
            return gx, gy

    raise RuntimeError("Failed to sample a free goal with requested distance/clearance.")


def send_goal(node, client: ActionClient, pose: PoseStamped, timeout_sec: float) -> int:
    goal = NavigateToPose.Goal()
    goal.pose = pose

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=timeout_sec)
    if not send_future.done():
        raise TimeoutError("Timed out waiting for goal acceptance.")
    goal_handle = send_future.result()
    if not goal_handle.accepted:
        return GoalStatus.STATUS_REJECTED

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout_sec)
    if not result_future.done():
        # Cancel so subsequent test iterations don't fight the still-running BT.
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
        raise TimeoutError("Timed out waiting for result (goal cancel requested).")
    return int(result_future.result().status)


def main():
    args = parse_args()
    rclpy.init(args=None)
    node = rclpy.create_node("go2_nav2_test_goal")

    if args.seed == 0:
        random.seed(int(time.time() * 1000) & 0xFFFFFFFF)
    else:
        random.seed(args.seed)

    tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
    tf_listener = TransformListener(tf_buffer, node)

    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    node.get_logger().info("Waiting for navigate_to_pose action server...")
    if not client.wait_for_server(timeout_sec=30.0):
        node.get_logger().error("navigate_to_pose action server not available.")
        rclpy.shutdown()
        sys.exit(1)

    successes = 0
    failures = 0

    for i in range(args.repeat):
        if args.auto:
            try:
                scan = wait_for_scan(node, timeout_sec=10.0)
                if not scan_has_returns(scan):
                    node.get_logger().warning(
                        "All /scan ranges are inf/nan (LiDAR may be publishing empty PointCloud2). "
                        "Mapping + obstacle layers will be degraded until valid returns appear."
                    )
            except TimeoutError:
                node.get_logger().warning(
                    "Timed out waiting for /scan (continuing). If this happens often, LiDAR output is unstable."
                )
            grid = wait_for_map(node, timeout_sec=args.map_wait)
            rx, ry, ryaw = lookup_xy_yaw(node, tf_buffer, args.frame, args.base_frame, timeout_sec=15.0)
            gx, gy = sample_goal_from_map(
                grid,
                robot_x=rx,
                robot_y=ry,
                min_distance=args.min_distance,
                max_distance=args.max_distance,
                clearance=args.clearance,
                allow_unknown=args.allow_unknown,
            )
            yaw_deg = math.degrees(ryaw)
        else:
            gx, gy, yaw_deg = args.x, args.y, args.yaw

        qx, qy, qz, qw = quat_from_yaw(math.radians(yaw_deg))
        pose = PoseStamped()
        pose.header.frame_id = args.frame
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        node.get_logger().info(
            f"[{i+1}/{args.repeat}] goal=({gx:.2f}, {gy:.2f}, yaw={yaw_deg:.1f}deg) frame='{args.frame}'"
        )
        t_start = time.time()
        try:
            status = send_goal(node, client, pose, timeout_sec=args.wait)
        except TimeoutError as e:
            failures += 1
            node.get_logger().error(f"[{i+1}/{args.repeat}] TIMEOUT after {time.time()-t_start:.1f}s: {e}")
            continue
        elapsed = time.time() - t_start

        if status == GoalStatus.STATUS_SUCCEEDED:
            successes += 1
            node.get_logger().info(f"[{i+1}/{args.repeat}] SUCCEEDED in {elapsed:.1f}s")
        else:
            failures += 1
            node.get_logger().error(f"[{i+1}/{args.repeat}] FAILED status={status} in {elapsed:.1f}s")

    node.get_logger().info(f"Summary: successes={successes}, failures={failures}")
    rclpy.shutdown()
    sys.exit(0 if failures == 0 else 5)


if __name__ == "__main__":
    main()
