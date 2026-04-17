"""Gazebo robot spawn/remove and per-robot subprocess management.

GazeboRobotManager mirrors GazeboMarkerManager (task.py) but for robot
entities.  RobotProcessManager launches the per-robot ros_gz_bridge and
robot_state_publisher as subprocesses (the runtime equivalent of what
spawn.launch.py does at launch time).
"""

import logging
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Dict

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

logger = logging.getLogger(__name__)


class GazeboRobotManager:
    """Spawn and remove robot entities in Gazebo via CLI."""

    WORLD_NAME = "warehouse_world"

    @staticmethod
    def _get_xacro_path() -> str:
        pkg_share = Path(get_package_share_directory("warehouse_gz"))
        return str(pkg_share / "models" / "simple_bot" / "robot.xacro")

    @staticmethod
    def process_xacro(robot_name: str) -> str:
        """Process the robot xacro template with the given name.

        Returns the URDF XML string (same as spawn.launch.py:126).
        """
        xacro_path = GazeboRobotManager._get_xacro_path()
        return xacro.process_file(
            xacro_path, mappings={"robot_name": robot_name}
        ).toxml()

    @staticmethod
    def _write_params_file(params: dict) -> str:
        """Write a ROS2 params YAML file for --params-file usage.

        Returns the path to the temporary file.  Caller is responsible
        for cleanup via os.unlink().
        """
        ros2_params = {"/**": {"ros__parameters": params}}
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False
        ) as f:
            yaml.dump(ros2_params, f, default_flow_style=False)
            return f.name

    @classmethod
    def spawn_robot(
        cls, name: str, x: float, y: float, yaw: float = 0.0
    ) -> bool:
        """Spawn a robot entity in Gazebo via ros_gz_sim create.

        Uses the same parameters as spawn.launch.py:128-147.
        Returns True on success.
        """
        robot_xml = cls.process_xacro(name)
        params = {
            "world": cls.WORLD_NAME,
            "string": robot_xml,
            "name": name,
            "use_sim_time": True,
            "allow_renaming": False,
            "x": float(x),
            "y": float(y),
            "z": 0.3,
            "R": 0.0,
            "P": 0.0,
            "Y": float(yaw),
        }
        params_file = cls._write_params_file(params)
        try:
            result = subprocess.run(
                [
                    "ros2", "run", "ros_gz_sim", "create",
                    "--ros-args",
                    "--params-file", params_file,
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if result.returncode != 0:
                logger.warning(
                    "Failed to spawn robot %s: %s", name, result.stderr
                )
                return False
            return True
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            logger.warning("Robot spawn error for %s: %s", name, e)
            return False
        finally:
            try:
                os.unlink(params_file)
            except OSError:
                pass

    @classmethod
    def remove_robot(cls, name: str) -> bool:
        """Remove a robot entity from Gazebo. Returns True on success.

        Same gz service pattern as GazeboMarkerManager.remove_marker.
        """
        try:
            result = subprocess.run(
                [
                    "gz", "service",
                    "-s", f"/world/{cls.WORLD_NAME}/remove",
                    "--reqtype", "gz.msgs.Entity",
                    "--reptype", "gz.msgs.Boolean",
                    "--req", f'name: "{name}" type: MODEL',
                    "--timeout", "5000",
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )
            if result.returncode != 0:
                logger.warning(
                    "Failed to remove robot %s: %s", name, result.stderr
                )
                return False
            return True
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            logger.warning("Robot remove error for %s: %s", name, e)
            return False


class RobotProcessManager:
    """Launch and track per-robot bridge + state_publisher subprocesses.

    Mirrors what spawn.launch.py:149-180 does via launch Nodes, but as
    subprocess.Popen so we can start/stop at runtime.
    """

    def __init__(self):
        self._processes: Dict[str, Dict[str, subprocess.Popen]] = {}
        self._param_files: Dict[str, str] = {}

    def start(self, name: str, robot_xml: str) -> None:
        """Start bridge and state_publisher for a robot.

        Args:
            name: Robot name (e.g. "robot_02").
            robot_xml: Processed URDF XML string from GazeboRobotManager.process_xacro.

        Raises:
            ValueError: If processes are already running for this robot.
        """
        if name in self._processes:
            raise ValueError(
                f"Processes already running for robot '{name}'"
            )

        # robot_state_publisher — same params as spawn.launch.py:149-161
        state_pub_params = {
            "robot_description": robot_xml,
            "use_sim_time": True,
            "frame_prefix": f"{name}/",
        }
        params_file = GazeboRobotManager._write_params_file(state_pub_params)
        self._param_files[name] = params_file

        state_pub_cmd = [
            "ros2", "run", "robot_state_publisher", "robot_state_publisher",
            "--ros-args",
            "-r", f"__ns:=/{name}",
            "--params-file", params_file,
        ]

        # ros_gz_bridge — same args/remappings as spawn.launch.py:163-180
        bridge_cmd = [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            f"/model/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"/model/{name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            f"/model/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "--ros-args",
            "-r", f"/model/{name}/cmd_vel:=/{name}/cmd_vel",
            "-r", f"/model/{name}/odom:=/{name}/odom",
            "-r", f"/model/{name}/joint_states:=/{name}/joint_states",
            "-r", f"/model/{name}/tf:=/tf",
        ]

        state_pub = subprocess.Popen(
            state_pub_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )

        bridge = subprocess.Popen(
            bridge_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )

        self._processes[name] = {"state_pub": state_pub, "bridge": bridge}
        logger.info(
            "Started subprocesses for %s (state_pub pid=%d, bridge pid=%d)",
            name, state_pub.pid, bridge.pid,
        )

        # Give processes a moment to start, then check they're alive
        time.sleep(1.0)
        for label, proc in self._processes[name].items():
            ret = proc.poll()
            if ret is not None:
                stderr_out = proc.stderr.read().decode() if proc.stderr else ""
                logger.warning(
                    "%s/%s exited immediately (code=%d): %s",
                    name, label, ret, stderr_out,
                )

    def stop(self, name: str, timeout: float = 3.0) -> None:
        """Stop bridge and state_publisher for a robot.

        Sends SIGTERM, waits, then SIGKILL if needed. No-op if not running.
        """
        procs = self._processes.pop(name, None)
        if procs is None:
            return

        for label, proc in procs.items():
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=timeout)
            except (ProcessLookupError, ChildProcessError):
                pass  # already dead
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    proc.wait(timeout=1.0)
                except (ProcessLookupError, ChildProcessError):
                    pass
            logger.info("Stopped %s/%s (pid=%d)", name, label, proc.pid)

        params_file = self._param_files.pop(name, None)
        if params_file is not None:
            try:
                os.unlink(params_file)
            except OSError:
                pass

    def stop_all(self) -> None:
        """Stop all managed subprocesses."""
        for name in list(self._processes.keys()):
            self.stop(name)
