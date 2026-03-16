"""Python API for controlling the robot fleet from external programs.

Usage::

    from warehouse_gz.fleet_client import FleetClient

    with FleetClient(robot_count=3) as fleet:
        fleet.send_goal("robot_00", x=3.0, y=4.0)
        pos = fleet.get_position("robot_00")   # (x, y, yaw)
        reached = fleet.wait_for_goal("robot_00", timeout=30.0)
"""

import math
import threading
import yaml
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FleetClient:
    """High-level Python interface to the robot fleet.

    Manages rclpy lifecycle internally so the caller does not need any
    ROS2 knowledge.  A background daemon thread keeps the node spinning.
    """

    def __init__(
        self,
        robot_names: Optional[List[str]] = None,
        robot_count: int = 3,
    ):
        try:
            pkg_share = Path(get_package_share_directory("warehouse_gz"))
            cfg_path = pkg_share / "config" / "warehouse.yaml"
            cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
            robot_count = cfg["spawn"]["robots"]
        except:
            raise ValueError(f"Malformed config file")
        
        if robot_names is None:
            robot_names = [f"robot_{i:02d}" for i in range(robot_count)]
        self._names = robot_names

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node("_fleet_client")
        self._node.set_parameters(
            [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
        )

        self._lock = threading.Lock()
        self._positions: Dict[str, Tuple[float, float, float]] = {}
        self._goal_reached: Dict[str, bool] = {}
        self._goal_events: Dict[str, threading.Event] = {}
        self._goal_pubs: Dict[str, rclpy.publisher.Publisher] = {}

        for name in self._names:
            self._goal_reached[name] = False
            self._goal_events[name] = threading.Event()

            self._node.create_subscription(
                Odometry,
                f"/{name}/odom",
                lambda msg, n=name: self._odom_cb(n, msg),
                10,
            )
            self._node.create_subscription(
                Bool,
                f"/{name}/goal_status",
                lambda msg, n=name: self._status_cb(n, msg),
                10,
            )
            self._goal_pubs[name] = self._node.create_publisher(
                PoseStamped, f"/{name}/goal_pose", 10
            )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True
        )
        self._spin_thread.start()

    # ---- callbacks ----

    def _odom_cb(self, name: str, msg: Odometry):
        pos = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        with self._lock:
            self._positions[name] = (pos.x, pos.y, yaw)

    def _status_cb(self, name: str, msg: Bool):
        with self._lock:
            self._goal_reached[name] = msg.data
        if msg.data:
            self._goal_events[name].set()

    # ---- public API ----

    def send_goal(self, robot_name: str, x: float, y: float) -> None:
        """Send a waypoint goal to a robot."""
        if robot_name not in self._goal_pubs:
            raise ValueError(f"Unknown robot: {robot_name}")
        with self._lock:
            self._goal_reached[robot_name] = False
        self._goal_events[robot_name].clear()

        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self._goal_pubs[robot_name].publish(msg)

    def get_position(self, robot_name: str) -> Tuple[float, float, float]:
        """Return ``(x, y, yaw)`` for a robot.  Raises if no odom received yet."""
        with self._lock:
            if robot_name not in self._positions:
                raise RuntimeError(
                    f"No odometry received yet for {robot_name}"
                )
            return self._positions[robot_name]

    def get_all_positions(self) -> Dict[str, Tuple[float, float, float]]:
        """Return positions for all robots that have reported odom."""
        with self._lock:
            return dict(self._positions)

    def wait_for_goal(self, robot_name: str, timeout: float = 30.0) -> bool:
        """Block until the robot reaches its goal or *timeout* seconds elapse.

        Returns ``True`` if the goal was reached, ``False`` on timeout.
        """
        return self._goal_events[robot_name].wait(timeout=timeout)

    def is_goal_reached(self, robot_name: str) -> bool:
        """Non-blocking goal status check."""
        with self._lock:
            return self._goal_reached.get(robot_name, False)

    # ---- lifecycle ----

    def shutdown(self) -> None:
        """Stop the background spin thread and clean up."""
        self._executor.shutdown()
        self._node.destroy_node()
        rclpy.try_shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.shutdown()
