import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Declare the launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Get the launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file with xacro
    pkg_path = get_package_share_directory('simmulator')
    xacro_file = os.path.join(pkg_path, 'description', 'iiwa7.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # Create the robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Create the joint_state_publisher_gui node
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )
    exe = ExecuteProcess(
            cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_factory.so'],
            output='screen')
    
    gazebo_launcher = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='gripper',
        output='screen',
        arguments=['-topic','/robot_description','-entity', 'gripper']
    )

    return LaunchDescription([
        declare_use_sim_time,
        node_robot_state_publisher,
        node_joint_state_publisher,
        gazebo_launcher,
        exe,
    ])
   