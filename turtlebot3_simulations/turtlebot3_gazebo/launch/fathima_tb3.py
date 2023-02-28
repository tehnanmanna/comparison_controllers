from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
import xacro
​
#### Successful code, however; the robot is not visible in gazebo it can be seen  un rviz only 
​
​
def generate_launch_description():
​
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    # Launch args
    pkg_share = FindPackageShare(package='warehouse_simulation').find('warehouse_simulation')
    world_file_name = 'warehouse_final3.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf_package=os.path.join(
        get_package_share_directory('turtlebot3_description'))
    urdf = os.path.join(
        urdf_package,
        'urdf',
        urdf_file_name)
​
   
 
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf],
    )
​
    node_joint_state=Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
     )
​
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )
​
    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
    )
​
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_tt',
        arguments=['-entity',
                   'turtlebot3_burger',
                 '-topic',
                   'robot_description',"-x", "7.0", "-y", "7.0", "-z", "0.0"],
        output='screen',
    )
​
​
    ld = LaunchDescription()
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_joint_state)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)
​
​
    return ld
​