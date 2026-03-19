import math
from pathlib import Path

import xacro
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from warehouse_gz.map_utils import (
    is_pose_clear_of_blocked_cells,
    load_blocked_cells,
    parse_octile_map,
)


def _make_grid_poses(n, xmin, xmax, ymin, ymax, spacing):
    poses = []
    x = xmin
    y = ymin
    while len(poses) < n and y <= ymax + 1e-9:
        while len(poses) < n and x <= xmax + 1e-9:
            poses.append((x, y, 0.3, 0.0, 0.0, 0.0))
            x += spacing
        x = xmin
        y += spacing
    return poses


def _resolve_map_path(context, cfg, cfg_path, pkg_share):
    map_override = LaunchConfiguration("map_file").perform(context).strip()
    map_file = map_override or str(cfg.get("map_file", "")).strip()
    if not map_file:
        raise RuntimeError(
            f"No map file selected. Set map_file launch arg or map_file in {cfg_path}"
        )

    map_path = pkg_share / "maps" / map_file
    if not map_path.is_file():
        raise RuntimeError(f"Map file not found: {map_path}")

    return map_path


def _launch(context, *args, **kwargs):
    pkg_share = Path(FindPackageShare("warehouse_gz").perform(context))
    cfg_path = pkg_share / "config" / "warehouse.yaml"
    cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}

    map_path = _resolve_map_path(context=context, cfg=cfg, cfg_path=cfg_path, pkg_share=pkg_share)
    map_width, map_height, rows = parse_octile_map(map_path)
    blocked_cells = load_blocked_cells(rows)

    resolution = float(cfg["resolution"])
    if resolution <= 0.0:
        raise RuntimeError(f"resolution must be > 0 in {cfg_path}, got {resolution}")

    origin = cfg.get("origin")
    if not isinstance(origin, (list, tuple)) or len(origin) != 2:
        raise RuntimeError(
            f"origin must be a two-element list in {cfg_path}, got {origin!r}"
        )
    origin_xy = (float(origin[0]), float(origin[1]))

    n = int(cfg["spawn"]["robots"])
    pattern = str(cfg["spawn"].get("pattern", "grid"))
    reg = cfg["spawn"]["start_region"]
    spacing = float(cfg["spawn"].get("robot_spacing", 0.8))
    robot_clearance = float(cfg["spawn"].get("robot_clearance", 1.1))

    xmin, xmax = float(reg["xmin"]), float(reg["xmax"])
    ymin, ymax = float(reg["ymin"]), float(reg["ymax"])

    if pattern != "grid":
        pattern = "grid"

    nx = int(math.floor((xmax - xmin) / spacing)) + 1
    ny = int(math.floor((ymax - ymin) / spacing)) + 1
    candidates = _make_grid_poses(nx * ny, xmin, xmax, ymin, ymax, spacing)
    poses = []
    for x, y, z, rr, pp, yy in candidates:
        is_free = is_pose_clear_of_blocked_cells(
            x=x,
            y=y,
            clearance=robot_clearance,
            blocked_cells=blocked_cells,
            origin_xy=origin_xy,
            resolution=resolution,
            map_height=map_height,
            map_width=map_width,
        )
        if is_free:
            poses.append((x, y, z, rr, pp, yy))
            if len(poses) >= n:
                break

    if len(poses) < n:
        raise RuntimeError(
            f"Not enough free space to place {n} robots in start_region "
            f"with spacing {spacing} and clearance {robot_clearance}. "
            f"Map: {map_path}"
        )

    model_xacro = str(pkg_share / "models" / "simple_bot" / "robot.xacro")
    actions = []

    for i in range(n):
        name = f"robot_{i:02d}"
        x, y, z, rr, pp, yy = poses[i]
        mappings = {"robot_name": name}
        robot_description = xacro.process_file(model_xacro, mappings=mappings).toxml()

        spawn_node = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            parameters=[
                {
                    "world": "warehouse_world",
                    "string": robot_description,
                    "name": name,
                    "use_sim_time": True,
                    "allow_renaming": False,
                    "x": x,
                    "y": y,
                    "z": z,
                    "R": rr,
                    "P": pp,
                    "Y": yy,
                }
            ],
        )

        node_robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=name,
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": True,
                    "frame_prefix": f"{name}/",
                }
            ],
        )

        bridge_node = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"{name}_bridge",
            output="screen",
            arguments=[
                f"/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                f"/model/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                f"/model/{name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                f"/model/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            ],
            remappings=[
                (f"/model/{name}/cmd_vel", f"/{name}/cmd_vel"),
                (f"/model/{name}/odom", f"/{name}/odom"),
                (f"/model/{name}/joint_states", f"/{name}/joint_states"),
                (f"/model/{name}/tf", "/tf"),
            ],
        )

        actions.append(
            TimerAction(
                period=0.5 * i,
                actions=[
                    LogInfo(msg=f"Spawning {name} at x={x:.3f}, y={y:.3f}, z={z:.3f}"),
                    spawn_node,
                    node_robot_state_publisher,
                    bridge_node,
                ],
            )
        )

    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )
    actions.append(clock_bridge_node)

    return actions


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
            OpaqueFunction(function=_launch),
        ]
    )
