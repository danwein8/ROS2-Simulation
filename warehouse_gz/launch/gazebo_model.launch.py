import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
import xacro
from pathlib import Path

def generate_launch_description():
    # this name has to match the robot name in the xacro file
    robot_xacro_name='simple_bot'

    # this is the name of our package, at the same time this is the name of the 
    # directory that will be used to define the paths
    pkg_name = "warehouse_gz"

    # this is a relative path to the xacro file defining the model
    model_relative_path = f"models/simple_bot/robot.xacro"

    # uncomment to define an empty world model 
    # TODO: I dont have this world, I have the warehouse world so Ill figure this out later
    # world_relative_path = f"worlds/empty.world"

    # absolute path to the model
    path_model_file = os.path.join(get_package_share_directory(pkg_name), model_relative_path)

    # this is the name of the world file, if you have one
    # path_world_file = os.path.join(get_package_share_directory(pkg_name), world_relative_path)

    # get the robot description from the xacro file
    robot_description = xacro.process_file(path_model_file).toxml()

    # launch file from the gazebo_ros package, this is the one that launches gazebo and spawns the robot
    gazebo_ros_launch_file = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))

    # launch description if you use your own world model
    # gazebo_launch = IncludeLaunchDescription(gazebo_ros_launch_file, launch_arguments={'gz_args': ['-r -v -v4', path_world_file], 'on_exit_shutdown': 'true'}.items())

    # launch description if you use the default world
    gazebo_launch = IncludeLaunchDescription(gazebo_ros_launch_file, launch_arguments={'gz_args': '-r -v -v4 empty.sdf', 'on_exit_shutdown': 'true'}.items())

    # these are ros nodes that we want to launch from this launch file
    # we add them to the launch description object later on, but we define them here for better readability
    # gazebo node
    spawn_model_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_xacro_name,
            '-topic', 'robot_description',
        ],
        output='screen',
    )

    # Robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

    # this is the node that bridges the topics between ROS and Gazebo, it uses the parameters defined in the bridge_parameters.yaml file
    bridge_params = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ],
        output='screen'
    )

    # here we create an empty launch description object
    ld = LaunchDescription()

    # add gazebo launch
    ld.add_action(gazebo_launch)

    # add the two nodes to the launch description
    ld.add_action(spawn_model_node)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(start_gazebo_ros_bridge)

    return ld