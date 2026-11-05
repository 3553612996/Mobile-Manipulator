#!/usr/bin/env python3
"""Send a Nav2 goal (and optionally an initial pose / chassis enable) from the CLI.

This is the repeatable half of the M5 acceptance test: RViz is nice for
exploring, but a navigation result needs a timestamp, a target and a distance
error. Run it with the robot on the floor, or with tools/bench/nav_sim.py to
validate the stack without hardware.

Examples:

    # real robot, localised at the mapping start pose (0, 0, 0)
    python3 tools/bench/nav_goal.py --enable --goal 1.5 -0.5 0

    # robot somewhere else: tell AMCL where it is first
    python3 tools/bench/nav_goal.py --initial-pose -1.0 -0.8 1.57 --goal 1.5 -0.5 0

    # offline smoke test
    python3 tools/bench/nav_sim.py --map .../maps/room2.yaml &
    python3 tools/bench/nav_goal.py --goal 1.5 -0.5 0

Exit code: 0 if the action returned SUCCEEDED, 1 for any other result or a
timeout. The final AMCL pose is printed so the caller can compute the arrival
error against the goal.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import SetBool, Trigger

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class NavGoalClient(Node):
    def __init__(self) -> None:
        super().__init__("nav_goal")
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        # AMCL publishes /amcl_pose RELIABLE + TRANSIENT_LOCAL. A VOLATILE
        # subscriber misses the latched pose and then waits forever while the
        # robot stands still (AMCL only republishes when it moves enough).
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, qos
        )
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.amcl_pose: PoseWithCovarianceStamped | None = None
        self.feedback = None
        self.result = None
        self._result_event = False

    # ------------------------------------------------------------- callbacks
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_pose = msg

    def _on_feedback(self, msg) -> None:
        self.feedback = msg.feedback

    def _on_result(self, future) -> None:
        self.result = future.result()
        self._result_event = True

    # ------------------------------------------------------------- helpers
    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def enable_chassis(self) -> bool:
        client = self.create_client(SetBool, "/set_enable")
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/set_enable not available")
            return False
        request = SetBool.Request()
        request.data = True
        future = client.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            self.get_logger().error("/set_enable timed out")
            return False
        response = future.result()
        self.get_logger().info(f"/set_enable success={response.success} message={response.message!r}")
        # base_bridge gates cmd_vel on REPORT_STATUS == MANUAL; next report <= 100 ms.
        self.spin_for(0.3)
        return bool(response.success)

    def wait_for_nav_active(self, timeout: float = 40.0) -> bool:
        """Wait for lifecycle_manager_navigation/is_active.

        An action server becomes discoverable before it is activated. Sending a
        goal during activation makes the BT fail immediately, which looks like a
        navigation bug in the logs. This is cheap to avoid.
        """
        client = self.create_client(Trigger, "/lifecycle_manager_navigation/is_active")
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                "/lifecycle_manager_navigation/is_active not available; "
                "sending the goal without the activation check"
            )
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.done() and future.result() is not None and future.result().success:
                self.get_logger().info("nav2 lifecycle is active")
                return True
            self.spin_for(0.5)
        self.get_logger().error("nav2 did not become active")
        return False

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        # 100 ms in the past: AMCL looks up the pose at this stamp, and the odom
        # TF buffer may not have a sample exactly at "now" yet.
        msg.header.stamp = (self.get_clock().now() - Duration(seconds=0.1)).to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # 0.5 m / 0.26 rad (1 sigma): roughly one lidar scan of uncertainty.
        for i, value in enumerate([0.25, 0.25, 0.0, 0.0, 0.0, 0.0685]):
            msg.pose.covariance[i * 6 + i] = value
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f"initial pose published: x={x:.2f} y={y:.2f} yaw={yaw:.2f}")

    def wait_for_amcl(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.amcl_pose is not None:
                return True
        return False

    def send_goal(self, x: float, y: float, yaw: float) -> bool:
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("navigate_to_pose action server not available")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f"goal: x={x:.2f} y={y:.2f} yaw={yaw:.2f}")
        send_future = self.action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)
        self.get_logger().info("goal accepted, waiting for result")
        return True

    def print_feedback(self, last_print: float, timeout: float) -> float:
        now = time.monotonic()
        if self.feedback is not None and now - last_print >= 1.0:
            f = self.feedback
            self.get_logger().info(
                f"distance_remaining={f.distance_remaining:.2f} m "
                f"time={f.navigation_time.sec}.{f.navigation_time.nanosec // 1000000:03d} s"
            )
            return now
        return last_print

    def final_pose(self) -> str:
        if self.amcl_pose is None:
            return "AMCL pose unavailable"
        p = self.amcl_pose.pose.pose.position
        q = self.amcl_pose.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return f"final amcl_pose: x={p.x:.3f} y={p.y:.3f} yaw={yaw:.3f}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--goal", nargs=3, type=float, metavar=("X", "Y", "YAW"),
                        required=True, help="goal in the map frame (metres, radians)")
    parser.add_argument("--initial-pose", nargs=3, type=float, metavar=("X", "Y", "YAW"),
                        help="publish /initialpose before sending the goal")
    parser.add_argument("--enable", action="store_true",
                        help="call /set_enable true before navigating (real robot)")
    parser.add_argument("--amcl-timeout", type=float, default=15.0,
                        help="seconds to wait for /amcl_pose (default 15)")
    parser.add_argument("--nav-active-timeout", type=float, default=40.0,
                        help="seconds to wait for lifecycle_manager_navigation/is_active")
    parser.add_argument("--timeout", type=float, default=180.0,
                        help="navigation timeout in seconds (default 180)")
    args = parser.parse_args()

    rclpy.init()
    node = NavGoalClient()
    try:
        if args.enable and not node.enable_chassis():
            return 1

        if not node.wait_for_nav_active(args.nav_active_timeout):
            return 1

        if args.initial_pose:
            node.publish_initial_pose(*args.initial_pose)
            if not node.wait_for_amcl(args.amcl_timeout):
                node.get_logger().error(
                    "no /amcl_pose yet; check nav2, /scan and TF. Sending the goal anyway."
                )
        elif not node.wait_for_amcl(args.amcl_timeout):
            node.get_logger().warn(
                "no /amcl_pose yet; the robot must already be localised or use --initial-pose"
            )

        if not node.send_goal(*args.goal):
            return 1

        start = time.monotonic()
        last_print = 0.0
        while not node._result_event and time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            last_print = node.print_feedback(last_print, args.timeout)

        if not node._result_event:
            node.get_logger().error(f"navigation timed out after {args.timeout:.0f} s")
            return 1

        status = node.result.status
        name = STATUS_NAMES.get(status, str(status))
        elapsed = time.monotonic() - start
        node.get_logger().info(f"result={name} elapsed={elapsed:.1f} s")
        node.get_logger().info(node.final_pose())
        return 0 if status == GoalStatus.STATUS_SUCCEEDED else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
