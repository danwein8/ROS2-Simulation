"""Python API for controlling the robot fleet from external programs.

Usage::

    from warehouse_gz.fleet_client import FleetClient

    with FleetClient(robot_count=3) as fleet:
        fleet.send_goal("robot_00", x=3.0, y=4.0)
        pos = fleet.get_position("robot_00")   # (x, y, yaw)
        reached = fleet.wait_for_goal("robot_00", timeout=30.0)
"""

import concurrent.futures
import copy
import math
import threading
import time
import yaml
import json
import datetime
from datetime import timedelta
from pathlib import Path as FilePath
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field

import rclpy
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory

try:
    from warehouse_gz.map_utils import (
        grid_cell_to_world_center,
        is_blocked_char,
        world_to_grid_cell,
        parse_octile_map,
    )
    from warehouse_gz.task import Task, TaskStatus, GazeboMarkerManager
except ImportError:
    from map_utils import grid_cell_to_world_center, is_blocked_char, world_to_grid_cell, parse_octile_map
    from task import Task, TaskStatus, GazeboMarkerManager


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

@dataclass
class Results:
    timestep: int
    # wallclock start time
    wall_clock_time: datetime.datetime
    # sim start time
    sim_time: Time
    # did robot x complete timestep i?
    complete: Dict[str, bool] = field(default_factory=dict)
    # time to complete (blocking behavior may cause increasing finishing times)
    completion_time: Dict[str, timedelta] = field(default_factory=dict)
    # how far from goal robot ended
    position_error: Dict[str, Tuple[float, float]] = field(default_factory=dict)


@dataclass
class TaskResults:
    task_id: str
    robot_name: str
    success: bool
    nav_time: float
    dwell_time: float
    total_time: float
    final_position: Optional[Tuple[float, float, float]] = None


class FleetClient:
    """High-level Python interface to the robot fleet.

    Manages rclpy lifecycle internally so the caller does not need any
    ROS2 knowledge.  A background daemon thread keeps the node spinning.
    """

    def __init__(
        self,
        robot_names: Optional[List[str]] = None,
        robot_count: int = 0,
    ):
        try:
            pkg_share = FilePath(get_package_share_directory("warehouse_gz"))
            cfg_path = pkg_share / "config" / "warehouse.yaml"
            cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
        except Exception:
            raise ValueError(f"Malformed config file")
        
        if robot_names is None:
            robot_count = cfg["spawn"]["robots"]
            robot_names = [f"robot_{i:02d}" for i in range(robot_count)]
        
        self._names = robot_names

        self._resolution = cfg["resolution"]
        self._origin = cfg["origin"]
        map_path = pkg_share / "maps" / cfg["map_file"]
        self._map_name = FilePath(map_path).name
        self._map_width, self._map_height, self._rows = parse_octile_map(map_path)

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

        self._tasks: Dict[str, Task] = {}
        self._task_lock = threading.Lock()

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
            # self._goal_pubs[name] = self._node.create_publisher(
            #     PoseStamped, f"/{name}/goal_pose", 10
            # )
            self._goal_pubs[name] = self._node.create_publisher(
                RosPath, f"/{name}/goal_path", 10
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

        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0

        msg = RosPath()
        msg.header.frame_id = "world"
        msg.poses = [pose]
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
        
    def get_map_info(self) -> Tuple[str, Tuple[int, int], float, List[float]]:
        """Returns the current simulations map information to ensure
        map information matches what is being run by the external program
        return: map filename, map dimensions, map resolution, and origin"""
        return (self._map_name, (self._map_height, self._map_width), self._resolution, self._origin)

    def wait_for_goal(self, robot_name: str, timeout: float = 10.0) -> bool:
        """Block until the robot reaches its goal or *timeout* seconds elapse.

        Returns ``True`` if the goal was reached, ``False`` on timeout.
        """
        return self._goal_events[robot_name].wait(timeout=timeout)

    def is_goal_reached(self, robot_name: str) -> bool:
        """Non-blocking goal status check."""
        with self._lock:
            return self._goal_reached.get(robot_name, False)
        
    def send_grid_goal(self, robot_name: str, row: int, col: int) -> None:
        position = grid_cell_to_world_center(row, col, self._origin, self._resolution, self._map_height)
        self.send_goal(robot_name, position[0], position[1])

    def get_grid_position(self, robot_name: str) -> Tuple[int, int]:
        position = self.get_position(robot_name)
        return world_to_grid_cell(position[0], position[1], self._origin, self._resolution, self._map_height, self._map_width)
    
    def execute_plan(self, plan: list[Dict]) -> List[Results]:
        """plan should be a list of timesteps and each timestep is a dict mapping 
        robot names to grid cells:
        plan = [
            {"robot_00":(3,4), "robot_01": (5,6)},
            {"robot_00":(3,5), "robot_01": (5,7)}
            ...
        ]
        """
        result_list = []
        for i, timestep in enumerate(plan):
            start = datetime.datetime.now()
            res = Results(i, start, self._node.get_clock().now())
            complete = {robot: True for robot in timestep}
            completion_time = {r: 0.0 for r in timestep}
            pos_error = {rb: 0.0 for rb in timestep}
            for robot_name, (row, col) in timestep.items():
                self.send_grid_goal(robot_name, row, col)
            # wait for all robots to finish before next timestep
            for robot_name, (row, col) in timestep.items():
                if not self.wait_for_goal(robot_name):
                    self._node.get_logger().info(f'Robot {robot_name} failed navigation at timestep {i}')
                    complete[robot_name] = False
                completion_time[robot_name] = datetime.datetime.now() - start
                curr_pos = self.get_position(robot_name)
                desired_pos = grid_cell_to_world_center(row, col, self._origin, self._resolution, self._map_height)
                pos_error[robot_name] = (curr_pos[0] - desired_pos[0], curr_pos[1] - desired_pos[1])
            res.complete = complete
            res.completion_time = completion_time
            res.position_error = pos_error
            result_list.append(res)
        return result_list

    def load_plan_from_file(self, path: str) -> List[Results]:
        """For stored plans that external programs create and write.
        Path should be a json file that has the format to feed into execute_plan()
        plan = [
            {"robot_00":(3,4), "robot_01": (5,6)},
            {"robot_00":(3,5), "robot_01": (5,7)}
            ...
        ]
        """
        with open(path, 'r') as file:
            data = json.load(file)
            return self.execute_plan(data)

    # ---- task API ----

    def _validate_task_location(self, row: int, col: int) -> None:
        """Raise ValueError if (row, col) is out of bounds or on a blocked cell."""
        if row < 0 or row >= self._map_height or col < 0 or col >= self._map_width:
            raise ValueError(
                f"Task location ({row}, {col}) is out of map bounds "
                f"(0..{self._map_height - 1}, 0..{self._map_width - 1})"
            )
        if is_blocked_char(self._rows[row][col]):
            raise ValueError(
                f"Task location ({row}, {col}) is on a blocked cell "
                f"(char='{self._rows[row][col]}')"
            )

    def create_task(
        self,
        task_id: str,
        row: int,
        col: int,
        dwell_time: float = 5.0,
    ) -> Task:
        """Create a task at a grid location and spawn its visual marker in Gazebo.

        Raises:
            ValueError: If task_id already exists, location is blocked, or out of bounds.
        """
        if dwell_time < 0:
            raise ValueError(f"dwell_time must be >= 0, got {dwell_time}")

        self._validate_task_location(row, col)

        world_x, world_y = grid_cell_to_world_center(
            row, col, self._origin, self._resolution, self._map_height
        )

        task = Task(
            task_id=task_id,
            row=row,
            col=col,
            dwell_time=dwell_time,
            world_x=world_x,
            world_y=world_y,
        )

        if GazeboMarkerManager.spawn_marker(task_id, world_x, world_y):
            task.marker_spawned = True
        else:
            self._node.get_logger().warn(
                f"Could not spawn Gazebo marker for task '{task_id}' "
                f"-- task still usable, just not visible"
            )

        with self._task_lock:
            if task_id in self._tasks:
                raise ValueError(f"Task '{task_id}' already exists")
            self._tasks[task_id] = task

        self._node.get_logger().info(
            f"Created task '{task_id}' at grid ({row},{col}) "
            f"world ({world_x:.2f},{world_y:.2f}), dwell={dwell_time}s"
        )
        return task

    def execute_task(
        self,
        robot_name: str,
        task_id: str,
        nav_timeout: float = 30.0,
    ) -> bool:
        """Send a robot to a task, dwell, and mark complete.

        Lifecycle: PENDING -> ASSIGNED -> (navigate) -> IN_PROGRESS -> (dwell)
        -> COMPLETED.  On navigation timeout: ASSIGNED -> FAILED (marker stays).

        Returns True if task completed, False if failed.

        Raises:
            ValueError: If robot_name or task_id is unknown, or task is not PENDING.
        """
        if robot_name not in self._goal_pubs:
            raise ValueError(f"Unknown robot: {robot_name}")

        with self._task_lock:
            if task_id not in self._tasks:
                raise ValueError(f"Unknown task: {task_id}")
            task = self._tasks[task_id]
            if task.status != TaskStatus.PENDING:
                raise ValueError(
                    f"Task '{task_id}' is {task.status.value}, expected pending"
                )
            task.status = TaskStatus.ASSIGNED
            task.assigned_robot = robot_name

        self.send_grid_goal(robot_name, task.row, task.col)
        self._node.get_logger().info(
            f"{robot_name}: navigating to task '{task_id}' at ({task.row},{task.col})"
        )

        reached = self.wait_for_goal(robot_name, timeout=nav_timeout)

        if not reached:
            with self._task_lock:
                task.status = TaskStatus.FAILED
            self._node.get_logger().warn(
                f"{robot_name}: FAILED task '{task_id}' -- navigation timeout "
                f"after {nav_timeout}s (marker stays visible)"
            )
            return False

        with self._task_lock:
            task.status = TaskStatus.IN_PROGRESS
        self._node.get_logger().info(
            f"{robot_name}: arrived at task '{task_id}', dwelling for {task.dwell_time}s"
        )

        if task.dwell_time > 0:
            time.sleep(task.dwell_time)

        with self._task_lock:
            task.status = TaskStatus.COMPLETED

        if task.marker_spawned:
            if GazeboMarkerManager.remove_marker(task_id):
                task.marker_spawned = False
            else:
                self._node.get_logger().warn(
                    f"Could not remove Gazebo marker for task '{task_id}'"
                )

        self._node.get_logger().info(f"{robot_name}: COMPLETED task '{task_id}'")
        return True

    def execute_task_plan(
        self,
        assignments: Dict[str, str],
        nav_timeout: float = 30.0,
    ) -> List[TaskResults]:
        """Execute a batch of robot-task assignments in parallel.

        Args:
            assignments: ``{robot_name: task_id}`` mapping.
            nav_timeout: Max seconds for each navigation.

        All robots navigate simultaneously.  Blocks until all complete or fail.
        """
        if not assignments:
            return []

        results: List[TaskResults] = []

        def _run_one(robot_name: str, task_id: str) -> TaskResults:
            start = time.monotonic()
            success = self.execute_task(robot_name, task_id, nav_timeout)
            elapsed = time.monotonic() - start
            with self._task_lock:
                dwell = self._tasks[task_id].dwell_time if success else 0
            pos = self.get_position(robot_name)
            return TaskResults(
                task_id=task_id,
                robot_name=robot_name,
                success=success,
                nav_time=elapsed - dwell,
                dwell_time=dwell,
                total_time=elapsed,
                final_position=pos,
            )

        with concurrent.futures.ThreadPoolExecutor(
            max_workers=len(assignments)
        ) as pool:
            futures = {
                pool.submit(_run_one, robot, task): (robot, task)
                for robot, task in assignments.items()
            }
            for future in concurrent.futures.as_completed(futures):
                results.append(future.result())

        return results

    def get_task(self, task_id: str) -> Task:
        """Return a copy of the task by ID."""
        with self._task_lock:
            if task_id not in self._tasks:
                raise ValueError(f"Unknown task: {task_id}")
            return copy.copy(self._tasks[task_id])

    def get_all_tasks(self) -> Dict[str, Task]:
        """Return a snapshot (copies) of all tasks."""
        with self._task_lock:
            return {k: copy.copy(v) for k, v in self._tasks.items()}

    def get_tasks_by_status(self, status: TaskStatus) -> List[Task]:
        """Return copies of tasks matching a given status."""
        with self._task_lock:
            return [copy.copy(t) for t in self._tasks.values() if t.status == status]

    def remove_task(self, task_id: str) -> None:
        """Remove a task and its Gazebo marker if present."""
        with self._task_lock:
            if task_id not in self._tasks:
                raise ValueError(f"Unknown task: {task_id}")
            task = self._tasks.pop(task_id)

        if task.marker_spawned:
            GazeboMarkerManager.remove_marker(task_id)

    # ---- lifecycle ----

    def shutdown(self) -> None:
        """Stop the background spin thread and clean up."""
        with self._task_lock:
            for task in self._tasks.values():
                if task.marker_spawned:
                    GazeboMarkerManager.remove_marker(task.task_id)
                    task.marker_spawned = False

        self._executor.shutdown()
        self._node.destroy_node()
        rclpy.try_shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.shutdown()
