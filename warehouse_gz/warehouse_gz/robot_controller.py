"""Fleet controller node: drives multiple robots to waypoint goals."""

import math
import yaml
from dataclasses import dataclass, field
from typing import Optional
from pathlib import Path
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory


# simple class to hold the current position from odom
# current active goal, and goal state (active, reached)
# one RobotState per robot
@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    waypoint_index: int = 0
    waypoint_queue: deque = field(default_factory=deque)
    goal_active: bool = False
    goal_reached: bool = False


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class FleetController(Node):
    def __init__(self):
        super().__init__("fleet_controller")

        pkg_share = Path(get_package_share_directory("warehouse_gz"))
        cfg_path = pkg_share / "config" / "warehouse.yaml"
        cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
        num_robs = cfg["spawn"]["robots"]

        self.declare_parameter("robot_count", num_robs)
        self.declare_parameter("linear_speed", 0.5)
        self.declare_parameter("angular_speed", 1.0)
        self.declare_parameter("goal_tolerance", 0.15)
        self.declare_parameter("heading_tolerance", 0.1)
        self.declare_parameter("collision_buffer", 0.8)

        robot_count = self.get_parameter("robot_count").value
        self._linear_speed = self.get_parameter("linear_speed").value
        self._angular_speed = self.get_parameter("angular_speed").value
        self._goal_tolerance = self.get_parameter("goal_tolerance").value
        self._heading_tolerance = self.get_parameter("heading_tolerance").value
        self._collision_buffer = self.get_parameter("collision_buffer").value

        self._robots: dict[str, RobotState] = {}
        self._cmd_pubs: dict[str, rclpy.publisher.Publisher] = {}
        self._status_pubs: dict[str, rclpy.publisher.Publisher] = {}

        # must create these subs and pubs for each robot
        for i in range(robot_count):
            name = f"robot_{i:02d}"
            self._robots[name] = RobotState()

            # get from Gazebo
            self.create_subscription(
                Odometry,
                f"/{name}/odom",
                lambda msg, n=name: self._odom_cb(n, msg),
                10,
            )
            # Path subscription
            self.create_subscription(
                RosPath,
                f"/{name}/goal_path",
                lambda msg, n=name: self._path_cb(n, msg),
                10,
            )

            # to Gazebo
            self._cmd_pubs[name] = self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
            # to FleetClient
            self._status_pubs[name] = self.create_publisher(Bool, f"/{name}/goal_status", 10)

        self.create_timer(0.05, self._control_loop)  # 20 Hz
        self.get_logger().info(
            f"Fleet controller started for {robot_count} robots"
        )

    def _odom_cb(self, name: str, msg: Odometry):
        state = self._robots[name]
        state.x = msg.pose.pose.position.x
        state.y = msg.pose.pose.position.y
        state.yaw = _yaw_from_quat(msg.pose.pose.orientation)
    
    def _path_cb(self, name: str, msg: RosPath):
        state = self._robots[name]
        state.waypoint_queue = deque((pose.pose.position.x, pose.pose.position.y) for pose in msg.poses)
        state.waypoint_index = 0
        state.goal_active = True
        state.goal_reached = False
        self._status_pubs[name].publish(Bool(data=False))
        self.get_logger().info(f"{name}: received path with {len(msg.poses)} waypoints")


    def _too_close(self, name: str) -> bool:
        state = self._robots[name]
        for other_name, other_state in self._robots.items():
            if other_name == name:
                continue
            if math.hypot(other_state.x - state.x, other_state.y - state.y) < self._collision_buffer:
                return True
        return False

    def _control_loop(self):
        for name, state in self._robots.items():
            if not state.goal_active or state.goal_reached:
                continue

            if self._too_close(name):
                self._cmd_pubs[name].publish(Twist())
                self.get_logger().warn(
                    f"{name}: collision buffer stop",
                    throttle_duration_sec=2.0,
                )
                continue

            # compute distance to goal
            goal_x, goal_y = state.waypoint_queue[state.waypoint_index]
            dx = goal_x - state.x
            dy = goal_y - state.y
            distance = math.hypot(dx, dy)

            # if within tolerance stop robot and publish goal status
            if distance < self._goal_tolerance:
                if state.waypoint_index >= len(state.waypoint_queue) - 1:
                    self._cmd_pubs[name].publish(Twist())
                    state.goal_reached = True
                    state.goal_active = False
                    self._status_pubs[name].publish(Bool(data=True))
                    self.get_logger().info(f"{name}: goal reached")
                else:
                    state.waypoint_index += 1
                continue

            # otherwise compute angle and heading error to face goal
            target_yaw = math.atan2(dy, dx)
            yaw_error = _normalize_angle(target_yaw - state.yaw)

            twist = Twist()
            # if heading error is large just turn
            if abs(yaw_error) > self._heading_tolerance:
                sign = 1.0 if yaw_error > 0 else -1.0
                twist.angular.z = sign * min(self._angular_speed, abs(yaw_error) * 2.0)
            # if its correct go forward, slowing as we get closer to avoid overshoot
            else:
                twist.linear.x = min(self._linear_speed, distance * 0.5)
                twist.angular.z = yaw_error * 2.0

            self._cmd_pubs[name].publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FleetController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
