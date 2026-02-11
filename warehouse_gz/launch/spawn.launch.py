import math
from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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

def _is_pose_free(x, y, shelves, clearance):
    for sh in shelves:
        sx = float(sh["x"])
        sy = float(sh["y"])
        half_w = 0.5 * float(sh["w"])
        half_h = 0.5 * float(sh["h"])
        if (
            (sx - half_w - clearance) <= x <= (sx + half_w + clearance)
            and (sy - half_h - clearance) <= y <= (sy + half_h + clearance)
        ):
            return False
    return True

def _launch(context, *args, **kwargs):
    pkg_share = Path(FindPackageShare("warehouse_gz").perform(context))
    cfg_path = pkg_share / "config" / "warehouse.yaml"
    cfg = yaml.safe_load(cfg_path.read_text())

    n = int(cfg["spawn"]["robots"])
    pattern = str(cfg["spawn"].get("pattern", "grid"))
    reg = cfg["spawn"]["start_region"]
    spacing = float(cfg["spawn"].get("robot_spacing", 0.8))
    robot_clearance = float(cfg["spawn"].get("robot_clearance", 1.1))
    shelves = cfg.get("shelves", [])

    xmin, xmax = float(reg["xmin"]), float(reg["xmax"])
    ymin, ymax = float(reg["ymin"]), float(reg["ymax"])

    if pattern != "grid":
        # keep it simple for now; grid is deterministic and debuggable
        pattern = "grid"

    nx = int(math.floor((xmax - xmin) / spacing)) + 1
    ny = int(math.floor((ymax - ymin) / spacing)) + 1
    candidates = _make_grid_poses(nx * ny, xmin, xmax, ymin, ymax, spacing)
    poses = []
    for x, y, z, rr, pp, yy in candidates:
        if _is_pose_free(x, y, shelves, robot_clearance):
            poses.append((x, y, z, rr, pp, yy))
            if len(poses) >= n:
                break
    if len(poses) < n:
        raise RuntimeError(
            f"Not enough free space to place {n} robots in start_region "
            f"with spacing {spacing} and clearance {robot_clearance}"
        )

    # Absolute path to the robot model SDF file
    model_sdf = str(pkg_share / "models" / "simple_bot" / "model.sdf")

    actions = []

    for i in range(n):
        name = f"robot_{i:02d}"
        x, y, z, rr, pp, yy = poses[i]

        # Use ros_gz_sim's "create" node to spawn the model.
        # It calls Gazebo's /world/<name>/create service internally
        # via native C++ gz-transport, avoiding the protobuf
        # text-format quoting issues we hit with the gz CLI.
        spawn_node = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            parameters=[{
                "world": "warehouse_world",
                "file": model_sdf,
                "name": name,
                "allow_renaming": True,
                "x": x,
                "y": y,
                "z": z,
                "R": rr,
                "P": pp,
                "Y": yy,
            }],
        )

        actions.append(TimerAction(
            period=0.5 * i,
            actions=[
                LogInfo(msg=f"Spawning {name} at x={x:.3f}, y={y:.3f}, z={z:.3f}"),
                spawn_node,
            ]
        ))

    return actions

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch)])
