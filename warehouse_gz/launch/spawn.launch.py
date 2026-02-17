import math
from pathlib import Path
import yaml
import os
import xacro

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

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
    pkg_name = "warehouse_gz"
    
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
    # model_sdf = str(pkg_share / "models" / "simple_bot" / "model.sdf")
    model_xacro = str(pkg_share / "models" / "simple_bot" / "robot.xacro")

    actions = []

    for i in range(n):
        name = f"robot_{i:02d}"
        x, y, z, rr, pp, yy = poses[i]
        mappings = {'robot_name': name}
        robot_description = xacro.process_file(model_xacro, mappings=mappings).toxml()

        spawn_node = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            parameters=[{
                "world": "warehouse_world",
                "string": robot_description,
                "name": name,
                "use_sim_time": True,
                "allow_renaming": False,
                # "namespace": name,
                # "frame_prefix": f"{name}/",
                "x": x,
                "y": y,
                "z": z,
                "R": rr,
                "P": pp,
                "Y": yy,
            }],
        )

        # Robot state publisher node
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True,
                         'frame_prefix': f"{name}/"}]
        )

    # this is the node that bridges the topics between ROS and Gazebo
        bridge_node = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"{name}_bridge",
            output="screen",
        arguments=[
            # bracket direction between msg types indicates direction of bridge
            # ros to gz ], gz to ros [
            # cmd_vel ROS Twist -> gz Twist
            f"/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            # odom gz Odometry -> ROS Odometry
            f"/model/{name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            # joint_states gz Model -> ROS JointState
            f"/model/{name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            # tf gz Pose_V -> ROS TFMessage
            f"/model/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[
            (f'/model/{name}/cmd_vel', f'/{name}/cmd_vel'),
            (f'/model/{name}/odom', f'/{name}/odom'),
            (f'/model/{name}/joint_states', f'/{name}/joint_states'),
            (f'/model/{name}/tf', f'/tf'),
        ],
    )
        
        actions.append(TimerAction(
            period=0.5 * i,
            actions=[
                LogInfo(msg=f"Spawning {name} at x={x:.3f}, y={y:.3f}, z={z:.3f}"),
                spawn_node,
                node_robot_state_publisher,
                bridge_node,
            ]
        ))
    
    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",],
    )

    # pose_info_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     name="world_pose_info_bridge",
    #     output="screen",
    #     arguments=[
    #         # tf gz Pose_V -> ROS TFMessage
    #         "world/warehouse_world/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
    #     ],
    # )

    actions.append(clock_bridge_node)
    # actions.append(pose_info_bridge)

    return actions

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch)])
