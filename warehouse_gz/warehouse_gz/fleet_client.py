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
from std_msgs.msg import Bool, String
from ament_index_python.packages import get_package_share_directory

try:
    from warehouse_gz.map_utils import (
        grid_cell_to_world_center,
        is_blocked_char,
        world_to_grid_cell,
        parse_octile_map,
    )
    from warehouse_gz.task import Task, TaskStatus, GazeboMarkerManager
    from warehouse_gz.robot_spawn import GazeboRobotManager, RobotProcessManager
except ImportError:
    from map_utils import grid_cell_to_world_center, is_blocked_char, world_to_grid_cell, parse_octile_map
    from task import Task, TaskStatus, GazeboMarkerManager
    from robot_spawn import GazeboRobotManager, RobotProcessManager


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
    dwelled: Dict[str, bool] = field(default_factory=dict)


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

    THERE ARE 3 LOCKS:
    `_robot_lock` → `_task_lock` → `_lock` — always acquire outer first
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
        self._odom_subs: Dict[str, rclpy.subscription.Subscription] = {}
        self._status_subs: Dict[str, rclpy.subscription.Subscription] = {}

        self._tasks: Dict[str, Task] = {}
        self._task_lock = threading.Lock()
        self._robot_task: Dict[str, Optional[str]] = {}
        self._task_exited: Dict[str, threading.Event] = {}

        # Dynamic robot management
        self._robot_lock = threading.Lock()
        self._robot_process_mgr = RobotProcessManager()
        self._dynamic_robots: set = set()
        self._robot_add_pub = self._node.create_publisher(
            String, "/fleet_controller/robot_add", 10
        )
        self._robot_remove_pub = self._node.create_publisher(
            String, "/fleet_controller/robot_remove", 10
        )

        for name in self._names:
            self._wire_robot(name)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True
        )
        self._spin_thread.start()

    # ---- per-robot wiring ----

    def _wire_robot(self, name: str) -> None:
        """Create ROS subs/pub and internal state for a single robot."""
        self._goal_reached[name] = False
        self._goal_events[name] = threading.Event()
        self._robot_task[name] = None
        self._task_exited[name] = threading.Event()

        self._odom_subs[name] = self._node.create_subscription(
            Odometry,
            f"/{name}/odom",
            lambda msg, n=name: self._odom_cb(n, msg),
            10,
        )
        self._status_subs[name] = self._node.create_subscription(
            Bool,
            f"/{name}/goal_status",
            lambda msg, n=name: self._status_cb(n, msg),
            10,
        )
        self._goal_pubs[name] = self._node.create_publisher(
            RosPath, f"/{name}/goal_path", 10
        )

    def _unwire_robot(self, name: str) -> None:
        """Destroy ROS subs/pub and internal state for a single robot."""
        sub = self._odom_subs.pop(name, None)
        if sub:
            self._node.destroy_subscription(sub)
        sub = self._status_subs.pop(name, None)
        if sub:
            self._node.destroy_subscription(sub)
        pub = self._goal_pubs.pop(name, None)
        if pub:
            self._node.destroy_publisher(pub)

        with self._lock:
            self._positions.pop(name, None)
            self._goal_reached.pop(name, None)
            self._goal_events.pop(name, None)
        with self._task_lock:
            self._robot_task.pop(name, None)
            self._task_exited.pop(name, None)

    # ---- dynamic robot add/remove ----

    def add_robot(self, name: str, row: int, col: int) -> None:
        """Spawn a new robot at a grid location and wire it up.

        Creates the Gazebo entity, starts bridge/state_publisher subprocesses,
        wires FleetClient subs/pubs, and notifies the FleetController.

        Raises:
            ValueError: If name already exists or location is invalid.
            RuntimeError: If Gazebo spawn fails or odom doesn't arrive.
        """
        with self._robot_lock:
            if name in self._names:
                raise ValueError(f"Robot '{name}' already exists")

        self._validate_task_location(row, col)

        world_x, world_y = grid_cell_to_world_center(
            row, col, self._origin, self._resolution, self._map_height
        )

        # 1. Spawn Gazebo entity
        if not GazeboRobotManager.spawn_robot(name, world_x, world_y):
            raise RuntimeError(f"Failed to spawn Gazebo entity for '{name}'")

        # Brief pause to let Gazebo register the model and start publishing topics
        time.sleep(2.0)

        # 2. Start bridge + state_publisher subprocesses
        robot_xml = GazeboRobotManager.process_xacro(name)
        try:
            self._robot_process_mgr.start(name, robot_xml)
        except Exception:
            GazeboRobotManager.remove_robot(name)
            raise

        # 3. Wire FleetClient subs/pubs
        self._wire_robot(name)

        with self._robot_lock:
            self._names.append(name)
            self._dynamic_robots.add(name)

        # 4. Notify FleetController
        self._robot_add_pub.publish(String(data=name))

        # 5. Wait for first odom so the robot is confirmed alive
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            with self._lock:
                if name in self._positions:
                    break
            time.sleep(0.2)
        else:
            self._node.get_logger().warn(
                f"No odom received for '{name}' within 20s — "
                f"robot may not be ready yet"
            )

        self._node.get_logger().info(
            f"Added robot '{name}' at grid ({row},{col}) "
            f"world ({world_x:.2f},{world_y:.2f})"
        )

    def remove_robot(self, name: str) -> None:
        """Remove a robot from the simulation and tear down all state.

        Cancels any in-progress task, notifies FleetController, kills
        subprocesses, and removes the Gazebo entity.

        Raises:
            ValueError: If robot name is unknown.
        """
        with self._robot_lock:
            if name not in self._names:
                raise ValueError(f"Unknown robot: {name}")

        # 1. Cancel any active task on this robot
        with self._task_lock:
            active_task_id = self._robot_task.get(name)
            if active_task_id and active_task_id in self._tasks:
                self._tasks[active_task_id].cancel_event.set()
            self._task_exited[name].wait(timeout=5.0)

        # 2. Notify FleetController to stop controlling this robot
        self._robot_remove_pub.publish(String(data=name))

        # 3. Tear down FleetClient ROS resources
        self._unwire_robot(name)

        # 4. Kill bridge + state_publisher subprocesses (if dynamic)
        self._robot_process_mgr.stop(name)

        # 5. Remove Gazebo entity
        GazeboRobotManager.remove_robot(name)

        with self._robot_lock:
            if name in self._names:
                self._names.remove(name)
            self._dynamic_robots.discard(name)

        self._node.get_logger().info(f"Removed robot '{name}'")

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
    
    @staticmethod
    def paths_to_timesteps(
            paths: Dict[str, List[Tuple[int, int]]]
    ) -> List[Dict[str, Tuple[int, int]]]:
        """convert {robot: [cells]} into [{robot: cell_at_t}, ...]
        
        Pads shorter paths by repeating the final cell so a robot that
        finishes early just sits at its goal while others catch up
        """
        if not paths:
            return []
        max_len = max(len(p) for p in paths.values())
        timesteps = []
        for t in range(max_len):
            step = {}
            for robot, cells in paths.items():
                step[robot] = cells[t] if t < len(cells) else cells[-1]
            timesteps.append(step)
        return timesteps


    def synchronized_path_follow(
            self,
            plan: list[Dict[str, Tuple[int, int]]],
            dwell_time: Optional[Dict[str, float]] = None,
            nav_timeout: float = 30.0,
            plan_cancel_event: Optional[threading.Event] = None,
            ) -> List[Results]:
        """Execute a MAPF plan with per-timestep synchronization.

        All robots present in a timestep must reach their cell before any
        robot advances to the next timestep.  Robots that appear/disappear
        between timesteps are added/removed from the sim.

        Args:
            plan: List of timesteps.  Each timestep is ``{robot_name: (row, col)}``.
            dwell_time: Optional ``{robot_name: seconds}`` applied after the
                full plan completes.  Robots dwell in parallel; dwell can be
                interrupted by ``plan_cancel_event``.
            nav_timeout: Per-timestep navigation timeout (seconds).
            plan_cancel_event: External cancellation flag.  When set, the
                current timestep wait is aborted and the function stops all
                active robots before returning.

        Returns:
            ``Results`` per executed timestep.  Dwell outcomes (if any) land
            in ``result_list[-1].dwelled``.
        """
        if plan_cancel_event is None:
            plan_cancel_event = threading.Event()
        if dwell_time is None:
            dwell_time = {}

        result_list: List[Results] = []
        plan_robots = set(name for timestep in plan for name in timestep)
        previous_robots = set(self._names) & plan_robots

        for i, timestep in enumerate(plan):
            if plan_cancel_event.is_set():
                self._node.get_logger().info(
                    f"Plan cancelled before timestep {i}"
                )
                break

            current_robots = set(timestep.keys())

            # Diff: robots that appeared or disappeared
            added = current_robots - previous_robots
            removed = previous_robots - current_robots

            for name in removed:
                self._node.get_logger().info(
                    f"Timestep {i}: removing robot '{name}'"
                )
                try:
                    self.remove_robot(name)
                except (ValueError, RuntimeError) as e:
                    self._node.get_logger().warn(
                        f"Failed to remove robot '{name}': {e}"
                    )

            for name in added:
                row, col = timestep[name]
                self._node.get_logger().info(
                    f"Timestep {i}: adding robot '{name}' at ({row},{col})"
                )
                try:
                    self.add_robot(name, row, col)
                except (ValueError, RuntimeError) as e:
                    self._node.get_logger().warn(
                        f"Failed to add robot '{name}': {e}"
                    )

            # failed adds wont poison the next robot diff
            previous_robots = set(self._names) & plan_robots

            # Execute the timestep goals (only for robots present)
            start = datetime.datetime.now()
            res = Results(i, start, self._node.get_clock().now())
            active = {k: v for k, v in timestep.items() if k in self._names}

            for robot_name, (row, col) in active.items():
                self.send_grid_goal(robot_name, row, col)

            complete: Dict[str, bool] = {}
            completion_time: Dict[str, timedelta] = {}
            pos_error: Dict[str, Tuple[float, float]] = {}

            def _wait_one(robot_name: str, row: int, col: int):
                reached = self._wait_for_goal_or_cancel(
                    robot_name, plan_cancel_event, nav_timeout
                )
                ct = datetime.datetime.now() - start
                try:
                    curr_pos = self.get_position(robot_name)
                    desired_pos = grid_cell_to_world_center(
                        row, col, self._origin, self._resolution, self._map_height
                    )
                    pe = (curr_pos[0] - desired_pos[0], curr_pos[1] - desired_pos[1])
                except RuntimeError:
                    pe = (float('inf'), float('inf'))
                return robot_name, reached, ct, pe

            if active:
                with concurrent.futures.ThreadPoolExecutor(
                    max_workers=len(active)
                ) as pool:
                    futures = [
                        pool.submit(_wait_one, name, row, col)
                        for name, (row, col) in active.items()
                    ]
                    for future in concurrent.futures.as_completed(futures):
                        name, reached, ct, pe = future.result()
                        complete[name] = reached
                        completion_time[name] = ct
                        pos_error[name] = pe
                        if not reached:
                            self._node.get_logger().info(
                                f"Robot {name} failed navigation at timestep {i}"
                            )

            res.complete = complete
            res.completion_time = completion_time
            res.position_error = pos_error
            result_list.append(res)

            if plan_cancel_event.is_set():
                for name in list(active.keys()):
                    self._stop_robot(name)
                break

        # Dwell phase — robots hold their final cell for their per-robot duration
        if dwell_time and not plan_cancel_event.is_set():
            dwellers = {
                r: d for r, d in dwell_time.items()
                if r in self._names and d > 0
            }
            dwelled: Dict[str, bool] = {}

            def _dwell_one(robot_name: str, duration: float):
                cancelled = plan_cancel_event.wait(timeout=duration)
                return robot_name, not cancelled

            if dwellers:
                with concurrent.futures.ThreadPoolExecutor(
                    max_workers=len(dwellers)
                ) as pool:
                    futures = [
                        pool.submit(_dwell_one, name, dur)
                        for name, dur in dwellers.items()
                    ]
                    for future in concurrent.futures.as_completed(futures):
                        name, ok = future.result()
                        dwelled[name] = ok

                if result_list:
                    result_list[-1].dwelled = dwelled

        return result_list

    def execute_plan(self, plan: list[Dict]) -> List[Results]:
        """Execute a plan, automatically adding/removing robots between timesteps.

        plan should be a list of timesteps and each timestep is a dict mapping
        robot names to grid cells:
        plan = [
            {"robot_00":(3,4), "robot_01": (5,6)},
            {"robot_00":(3,5)},                      # robot_01 removed
            {"robot_00":(3,6), "robot_02": (1,1)},   # robot_02 added at (1,1)
        ]

        If a robot appears in a timestep but wasn't in the previous one,
        it is spawned at its grid location.  If a robot disappears, it is
        removed from the simulation.
        """
        result_list = []
        previous_robots: set = set(self._names)

        for i, timestep in enumerate(plan):
            current_robots = set(timestep.keys())

            # Diff: robots that appeared or disappeared
            added = current_robots - previous_robots
            removed = previous_robots - current_robots

            for name in removed:
                self._node.get_logger().info(
                    f"Timestep {i}: removing robot '{name}'"
                )
                try:
                    self.remove_robot(name)
                except (ValueError, RuntimeError) as e:
                    self._node.get_logger().warn(
                        f"Failed to remove robot '{name}': {e}"
                    )

            for name in added:
                row, col = timestep[name]
                self._node.get_logger().info(
                    f"Timestep {i}: adding robot '{name}' at ({row},{col})"
                )
                try:
                    self.add_robot(name, row, col)
                except (ValueError, RuntimeError) as e:
                    self._node.get_logger().warn(
                        f"Failed to add robot '{name}': {e}"
                    )

            # Execute the timestep goals (only for robots present)
            start = datetime.datetime.now()
            res = Results(i, start, self._node.get_clock().now())
            active = {k: v for k, v in timestep.items() if k in self._names}
            complete = {robot: True for robot in active}
            completion_time = {}
            pos_error = {rb: 0.0 for rb in active}
            for robot_name, (row, col) in active.items():
                self.send_grid_goal(robot_name, row, col)
            # wait for all robots to finish before next timestep
            for robot_name, (row, col) in active.items():
                if not self.wait_for_goal(robot_name):
                    self._node.get_logger().info(f'Robot {robot_name} failed navigation at timestep {i}')
                    complete[robot_name] = False
                completion_time[robot_name] = datetime.datetime.now() - start
                try:
                    curr_pos = self.get_position(robot_name)
                    desired_pos = grid_cell_to_world_center(row, col, self._origin, self._resolution, self._map_height)
                    pos_error[robot_name] = (curr_pos[0] - desired_pos[0], curr_pos[1] - desired_pos[1])
                except RuntimeError:
                    pos_error[robot_name] = (float('inf'), float('inf'))
            res.complete = complete
            res.completion_time = completion_time
            res.position_error = pos_error
            result_list.append(res)

            previous_robots = set(name for name in current_robots if name in self._names)

        return result_list

    def load_plan_from_file(self, path: str) -> List[Results]:
        """For stored plans that external programs create and write.
        Path should be a json file that has the format to feed into execute_plan()
        can be tuples or lists for coordinates
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

        # Insert into dict BEFORE spawning marker to prevent race condition
        # where two concurrent create_task calls with the same ID both spawn
        # markers before either checks for duplicates.
        with self._task_lock:
            if task_id in self._tasks:
                raise ValueError(f"Task '{task_id}' already exists")
            self._tasks[task_id] = task

        if GazeboMarkerManager.spawn_marker(task_id, world_x, world_y):
            with self._task_lock:
                task.marker_spawned = True
        else:
            self._node.get_logger().warn(
                f"Could not spawn Gazebo marker for task '{task_id}' "
                f"-- task still usable, just not visible"
            )

        self._node.get_logger().info(
            f"Created task '{task_id}' at grid ({row},{col}) "
            f"world ({world_x:.2f},{world_y:.2f}), dwell={dwell_time}s"
        )
        return task

    def _wait_for_goal_or_cancel(
        self,
        robot_name: str,
        cancel_event: threading.Event,
        timeout: float = 10.0,
    ) -> bool:
        """Wait for goal reached OR cancellation, whichever comes first.

        Returns True if goal reached, False if timeout or cancelled.
        """
        deadline = time.monotonic() + timeout
        goal_event = self._goal_events[robot_name]
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            if goal_event.wait(timeout=min(0.1, remaining)):
                return True
            if cancel_event.is_set():
                return False

    def _stop_robot(self, robot_name: str) -> None:
        """Stop a robot by sending a goal at its current position."""
        try:
            pos = self.get_position(robot_name)
            self.send_goal(robot_name, pos[0], pos[1])
        except RuntimeError:
            pass  # no odom yet, robot will keep moving

    def execute_task(
        self,
        robot_name: str,
        task_id: str,
        nav_timeout: float = 30.0,
    ) -> bool:
        """Send a robot to a task, dwell, and mark complete.

        Lifecycle: PENDING -> ASSIGNED -> (navigate) -> IN_PROGRESS -> (dwell)
        -> COMPLETED.  On navigation timeout: ASSIGNED -> FAILED (marker stays).
        On cancellation (via remove_task): -> CANCELLED.

        Returns True if task completed, False if failed or cancelled.

        Raises:
            ValueError: If robot_name or task_id is unknown, task is not
                PENDING, or the robot is already busy.
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
            if self._robot_task.get(robot_name) is not None:
                raise ValueError(
                    f"Robot '{robot_name}' is busy with task "
                    f"'{self._robot_task[robot_name]}'"
                )
            task.status = TaskStatus.ASSIGNED
            task.assigned_robot = robot_name
            self._robot_task[robot_name] = task_id
            cancel_event = task.cancel_event

        try:
            # Navigation phase
            self.send_grid_goal(robot_name, task.row, task.col)
            self._node.get_logger().info(
                f"{robot_name}: navigating to task '{task_id}' "
                f"at ({task.row},{task.col})"
            )

            reached = self._wait_for_goal_or_cancel(
                robot_name, cancel_event, nav_timeout
            )

            if cancel_event.is_set():
                task.status = TaskStatus.CANCELLED
                self._stop_robot(robot_name)
                self._node.get_logger().info(
                    f"{robot_name}: CANCELLED task '{task_id}' during navigation"
                )
                return False

            if not reached:
                task.status = TaskStatus.FAILED
                self._node.get_logger().warn(
                    f"{robot_name}: FAILED task '{task_id}' -- navigation "
                    f"timeout after {nav_timeout}s (marker stays visible)"
                )
                return False

            # Dwell phase
            with self._task_lock:
                if cancel_event.is_set():
                    task.status = TaskStatus.CANCELLED
                    self._node.get_logger().info(
                        f"{robot_name}: CANCELLED task '{task_id}' before dwell"
                    )
                    return False
                task.status = TaskStatus.IN_PROGRESS

            self._node.get_logger().info(
                f"{robot_name}: arrived at task '{task_id}', "
                f"dwelling for {task.dwell_time}s"
            )

            if task.dwell_time > 0:
                cancelled = cancel_event.wait(timeout=task.dwell_time)
                if cancelled:
                    task.status = TaskStatus.CANCELLED
                    self._node.get_logger().info(
                        f"{robot_name}: CANCELLED task '{task_id}' during dwell"
                    )
                    return False

            # Completion
            with self._task_lock:
                if cancel_event.is_set():
                    task.status = TaskStatus.CANCELLED
                    return False
                task.status = TaskStatus.COMPLETED

            if task.marker_spawned:
                if GazeboMarkerManager.remove_marker(task_id):
                    task.marker_spawned = False
                else:
                    self._node.get_logger().warn(
                        f"Could not remove Gazebo marker for task '{task_id}'"
                    )

            self._node.get_logger().info(
                f"{robot_name}: COMPLETED task '{task_id}'"
            )
            return True
        finally:
            with self._task_lock:
                if self._robot_task.get(robot_name) == task_id:
                    self._robot_task[robot_name] = None
                    self._task_exited[robot_name].set()

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
                task_obj = self._tasks.get(task_id)
                dwell = task_obj.dwell_time if (task_obj and success) else 0.0
            try:
                pos = self.get_position(robot_name)
            except:
                pos = None
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
        """Remove a task and its Gazebo marker.  If in-progress, cancels first."""
        with self._task_lock:
            if task_id not in self._tasks:
                raise ValueError(f"Unknown task: {task_id}")
            task = self._tasks.pop(task_id)
            task.cancel_event.set()
            if (
                task.assigned_robot
                and self._robot_task.get(task.assigned_robot) == task_id
            ):
                self._robot_task[task.assigned_robot] = None

        if task.marker_spawned:
            GazeboMarkerManager.remove_marker(task_id)

    def clear_finished_tasks(self) -> int:
        """Remove all COMPLETED, FAILED, and CANCELLED tasks.  Returns count."""
        terminal = {TaskStatus.COMPLETED, TaskStatus.FAILED, TaskStatus.CANCELLED}
        with self._task_lock:
            to_remove = [t for t in self._tasks.values() if t.status in terminal]
            for task in to_remove:
                del self._tasks[task.task_id]
        for task in to_remove:
            if task.marker_spawned:
                GazeboMarkerManager.remove_marker(task.task_id)
        return len(to_remove)

    # ---- lifecycle ----

    def shutdown(self) -> None:
        """Stop the background spin thread and clean up."""
        # Cancel all in-flight tasks
        with self._task_lock:
            for task in self._tasks.values():
                task.cancel_event.set()
                if task.marker_spawned:
                    GazeboMarkerManager.remove_marker(task.task_id)
                    task.marker_spawned = False

        # Remove dynamic robots (kills subprocesses + Gazebo entities)
        for name in list(self._dynamic_robots):
            try:
                self.remove_robot(name)
            except (ValueError, RuntimeError):
                pass
        self._robot_process_mgr.stop_all()

        self._executor.shutdown()
        self._node.destroy_node()
        rclpy.try_shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.shutdown()
