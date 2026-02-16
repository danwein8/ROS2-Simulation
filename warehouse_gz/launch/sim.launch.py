import os
from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def _launch(context, *args, **kwargs):
    # generate the world file from the yaml config
    pkg_share = Path(FindPackageShare("warehouse_gz").perform(context))
    cfg_path = pkg_share / "config" / "warehouse.yaml"

    # create the worlds directory if it doesn't exist, and define the path to the world file
    worlds_dir = pkg_share / "worlds"
    worlds_dir.mkdir(parents=True, exist_ok=True)
    world_path = worlds_dir / "warehouse_world.sdf"

    # command to generate the world file from the yaml config
    gen_cmd = [
        "python3",
        str(pkg_share / "warehouse_gz" / "gen_world.py"),
        "--yaml", str(cfg_path),
        "--out", str(world_path),
    ]

    # command to launch gz sim with the generated world file
    gazebo_cmd = ["gz", "sim", "-r", str(world_path)]

    # spawn the robots after a short delay to give gz sim time to start up and load the world
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_share / "launch" / "spawn.launch.py"))
    )

    return [
        ExecuteProcess(cmd=gen_cmd, output="screen"),
        ExecuteProcess(cmd=gazebo_cmd, output="screen"),
        # give gz sim a moment to bring up the world services
        TimerAction(period=2.0, actions=[spawn_launch]),
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch)])
