#!/usr/bin/env python3
"""Minimal fake base + map raycaster for offline Nav2 smoke tests.

It is NOT a physics simulator and it is not used on the real robot. It exists
so the M5 stack can be exercised end to end without the chassis:

  true pose (map) ──raycast map──► /scan
  /cmd_vel ──differential integration──► /odom + TF odom→base_link
  base_link→laser_link static TF (URDF values)
  /initialpose published until AMCL answers

Assumptions, deliberately simple:
  * the robot follows /cmd_vel exactly (no slip, no acceleration limit);
  * the true pose equals the odom pose, so AMCL's map→odom should stay near
    identity. Any map→odom correction is real AMCL feedback;
  * unknown pixels in a trinary map are treated as free, matching map_server.

Example (three terminals, or run the first two in the background):

  ros2 launch robot_bringup navigation.launch.py rviz:=false
  python3 tools/bench/nav_sim.py \
      --map mobile_manipulator_ws/src/robot_bringup/maps/room2.yaml
  python3 tools/bench/nav_goal.py --goal 1.5 -0.5 0

If navigation.launch.py is not up yet, nav_sim keeps republishing the initial
pose until /amcl_pose arrives, so starting it first is also fine.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# URDF robot.urdf.xacro: laser_x = 0.245/2 - 0.060, laser_z = 0.050 + 0.0503 - 0.0079
DEFAULT_LASER_X = 0.0625
DEFAULT_LASER_Z = 0.0924


class MapRaycaster:
    """Occupancy grid + vectorized 2D ray casting (one numpy step per range)."""

    def __init__(self, yaml_path: Path) -> None:
        meta = yaml.safe_load(yaml_path.read_text())
        image_path = (yaml_path.parent / meta["image"]).resolve()
        image = np.array(Image.open(image_path).convert("L"))
        occ = image.astype(float) / 255.0
        if not meta.get("negate", 0):
            occ = 1.0 - occ
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        self.occupied = occ > occupied_thresh
        self.height, self.width = self.occupied.shape
        self.resolution = float(meta["resolution"])
        self.origin_x, self.origin_y = float(meta["origin"][0]), float(meta["origin"][1])
        self.max_x = self.origin_x + self.width * self.resolution
        self.max_y = self.origin_y + self.height * self.resolution

    def _occupied_at(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        cols = np.floor((xs - self.origin_x) / self.resolution).astype(int)
        rows = np.floor((self.max_y - ys) / self.resolution).astype(int)
        inside = (
            (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)
        )
        # Leaving the map bounds counts as a hit so the ray terminates.
        hit = np.ones(xs.shape, dtype=bool)
        hit[inside] = self.occupied[rows[inside], cols[inside]]
        return hit

    def scan(self, x: float, y: float, yaw: float, bins: int,
             range_min: float, range_max: float) -> np.ndarray:
        angles = -math.pi + np.arange(bins) * (2.0 * math.pi / bins)
        world = yaw + angles
        cos_a, sin_a = np.cos(world), np.sin(world)
        # The scan is taken at laser_link, not base_link.
        laser_x = x + DEFAULT_LASER_X * math.cos(yaw)
        laser_y = y + DEFAULT_LASER_X * math.sin(yaw)

        ranges = np.full(bins, np.inf, dtype=float)
        hit = np.zeros(bins, dtype=bool)
        step = self.resolution * 0.5
        for distance in np.arange(range_min, range_max, step):
            active = ~hit
            if not active.any():
                break
            xs = laser_x + distance * cos_a[active]
            ys = laser_y + distance * sin_a[active]
            newly_hit = self._occupied_at(xs, ys)
            indices = np.flatnonzero(active)[newly_hit]
            ranges[indices] = distance
            hit[indices] = True
        return np.minimum(ranges, range_max)


class FakeBase(Node):
    def __init__(self, args) -> None:
        super().__init__("fake_base")
        self.map = MapRaycaster(Path(args.map).resolve())
        self.x, self.y, self.yaw = args.start
        self.initial_pose = (self.x, self.y, self.yaw)

        self.bins = args.scan_bins
        self.range_min = 0.12
        self.range_max = 8.0
        self.scan_period = 1.0 / args.scan_rate
        self.scan_freq = args.scan_rate
        self.noise = args.noise
        self.rng = np.random.default_rng(args.seed)

        self.cmd = Twist()
        self.cmd_stamp = 0.0
        self.last_odom = self.get_clock().now()
        self.amcl_seen = False

        scan_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.scan_pub = self.create_publisher(LaserScan, "/scan", scan_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        # Match AMCL's latched QoS so the first pose is seen even before motion.
        amcl_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, amcl_qos
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self._publish_static_laser_tf()

        self.create_timer(1.0 / 50.0, self._odom_tick)
        self.create_timer(self.scan_period, self._scan_tick)
        self.create_timer(0.5, self._initial_pose_tick)
        self.create_timer(1.0, self._status_tick)
        self.get_logger().info(
            f"fake base at ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f}); "
            f"map {self.map.width}x{self.map.height} @ {self.map.resolution} m"
        )

    # ------------------------------------------------------------- callbacks
    def _on_cmd(self, msg: Twist) -> None:
        self.cmd = msg
        self.cmd_stamp = time.monotonic()

    def _on_amcl(self, _msg) -> None:
        if not self.amcl_seen:
            self.amcl_seen = True
            self.get_logger().info("AMCL is publishing /amcl_pose; stop initial-pose retries")

    def _publish_static_laser_tf(self) -> None:
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "laser_link"
        tf.transform.translation.x = DEFAULT_LASER_X
        tf.transform.translation.z = DEFAULT_LASER_Z
        tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform(tf)

    # ---------------------------------------------------------------- timers
    def _odom_tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_odom).nanoseconds / 1e9
        self.last_odom = now
        if dt <= 0.0:
            return
        if time.monotonic() - self.cmd_stamp > 0.5:
            # Mirror base_bridge's cmd_vel watchdog.
            v, w = 0.0, 0.0
        else:
            v, w = self.cmd.linear.x, self.cmd.angular.z
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += w * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        stamp = now.to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        for i, value in enumerate([0.01, 0.01, 0.0, 0.0, 0.0, 0.01]):
            odom.pose.covariance[i * 6 + i] = value
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        for i, value in enumerate([0.01, 0.01, 0.0, 0.0, 0.0, 0.01]):
            odom.twist.covariance[i * 6 + i] = value
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = math.sin(self.yaw / 2.0)
        tf.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf)

    def _scan_tick(self) -> None:
        ranges = self.map.scan(
            self.x, self.y, self.yaw, self.bins, self.range_min, self.range_max
        )
        if self.noise > 0.0:
            finite = np.isfinite(ranges)
            ranges[finite] += self.rng.normal(0.0, self.noise, finite.sum())
            ranges = np.clip(ranges, self.range_min, self.range_max)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser_link"
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / self.bins
        scan.angle_max = -math.pi + (self.bins - 1) * scan.angle_increment
        scan.time_increment = (1.0 / self.scan_freq) / self.bins
        scan.scan_time = 1.0 / self.scan_freq
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.astype(np.float32).tolist()
        self.scan_pub.publish(scan)

    def _initial_pose_tick(self) -> None:
        if self.amcl_seen:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = (self.get_clock().now() - Duration(seconds=0.1)).to_msg()
        msg.header.frame_id = "map"
        x, y, yaw = self.initial_pose
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        for i, value in enumerate([0.25, 0.25, 0.0, 0.0, 0.0, 0.0685]):
            msg.pose.covariance[i * 6 + i] = value
        self.initial_pub.publish(msg)

    def _status_tick(self) -> None:
        v = self.cmd.linear.x if time.monotonic() - self.cmd_stamp <= 0.5 else 0.0
        self.get_logger().info(
            f"pose=({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f}) "
            f"cmd_v={v:.2f} amcl={'up' if self.amcl_seen else 'waiting'}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--map", required=True, help="map YAML (room2.yaml)")
    parser.add_argument("--start", nargs=3, type=float, default=(0.0, 0.0, 0.0),
                        metavar=("X", "Y", "YAW"),
                        help="true/odom start pose in the map frame")
    parser.add_argument("--scan-bins", type=int, default=720)
    parser.add_argument("--scan-rate", type=float, default=6.0)
    parser.add_argument("--noise", type=float, default=0.01,
                        help="range noise sigma in metres; 0 disables")
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    rclpy.init()
    node = FakeBase(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
