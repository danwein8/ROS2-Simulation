from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch(context, *args, **kwargs):
    # generate the world file from the yaml config and map file
    pkg_share = Path(FindPackageShare("warehouse_gz").perform(context))
    cfg_path = pkg_share / "config" / "warehouse.yaml"
    cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}

    # resolved now bc its used here
    map_override = LaunchConfiguration("map_file").perform(context).strip()
    map_file = map_override or str(cfg.get("map_file", "")).strip()
    if not map_file:
        raise RuntimeError(
            f"No map file selected. Set map_file launch arg or map_file in {cfg_path}"
        )

    map_path = pkg_share / "maps" / map_file
    if not map_path.is_file():
        raise RuntimeError(f"Map file not found: {map_path}")
    
    # define then overwrite if start_positions is passed in
    n = int(cfg["spawn"]["robots"])
    start_pos_override = LaunchConfiguration("start_positions").perform(context).strip()
    if start_pos_override:
        pairs = start_pos_override.split(";")
        n = len(pairs)

    # create worlds dir if it doesn't exist, and set the world path
    worlds_dir = pkg_share / "worlds"
    worlds_dir.mkdir(parents=True, exist_ok=True)
    world_path = worlds_dir / "warehouse_world.sdf"

    # command to generate the world file from the config and map
    gen_cmd = [
        "python3",
        str(pkg_share / "warehouse_gz" / "gen_world.py"),
        "--yaml",
        str(cfg_path),
        "--map",
        str(map_path),
        "--out", 
        str(world_path),
    ]

    # command to launch gazebo with the generated world file
    gazebo_cmd = ["gz", "sim", "-r", str(world_path)]

    # spawn the robots after a delay to allow gazebo to start up
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_share / "launch" / "spawn.launch.py")),
        launch_arguments={
            "map_file": map_file,
            "start_positions": LaunchConfiguration("start_positions"),
        }.items(),
    )

    gen_process = ExecuteProcess(cmd=gen_cmd, output="screen")
    gz_process = ExecuteProcess(cmd=gazebo_cmd, output="screen")

    controller_node = Node(
        package="warehouse_gz",
        executable="fleet_controller",
        name="fleet_controller",
        output="screen",
        parameters=[{
            "robot_count": n,
            "use_sim_time": True,
        }],
    )

    return [
        gen_process,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gen_process,
                on_exit=[gz_process],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gen_process,
                on_exit=[TimerAction(period=5.0, actions=[spawn_launch])],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gen_process,
                on_exit=[TimerAction(period=8.0, actions=[controller_node])],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_file",
                default_value="",
                description=(
                    "Optional map filename from warehouse_gz/maps. "
                    "If empty, use config/warehouse.yaml map_file."
                ),
            ),
            DeclareLaunchArgument(
                "start_positions",
                default_value="",
                description=(
                    "Optional agent start positions from outside program. "
                    "If empty, use default grid pattern generation."
                ),
            ),
            OpaqueFunction(function=_launch),
        ]
    )
