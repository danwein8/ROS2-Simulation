import os
from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare

def _launch(context, *args, **kwargs):
    pkg_share = Path(FindPackageShare("warehouse_gz").perform(context))
    cfg_path = pkg_share / "config" / "warehouse.yaml"
    cfg = yaml.safe_load(cfg_path.read_text())

    # 1) generate world.sdf into package share "worlds/"
    worlds_dir = pkg_share / "worlds"
    worlds_dir.mkdir(parents=True, exist_ok=True)
    world_path = worlds_dir / "warehouse_world.sdf"

    gen_cmd = [
        "python3",
        str(pkg_share / "warehouse_gz" / "gen_world.py"),
        "--yaml", str(cfg_path),
        "--out", str(world_path),
    ]

    # 2) Start Gazebo Harmonic (gz sim)
    gazebo_cmd = ["gz", "sim", "-r", str(world_path)]

    actions = [
        ExecuteProcess(cmd=gen_cmd, output="screen"),
        ExecuteProcess(cmd=gazebo_cmd, output="screen"),
    ]

    # NOTE: spawning + bridging comes next; we’ll add it after build sanity check.
    # It’s better to verify the world launches cleanly before layering complexity.

    return actions

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch)])

