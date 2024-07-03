# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource


# from launch_ros.actions import Node
# import xacro


# def generate_launch_description():

#     # Specify the name of the package and path to xacro file within the package
#     pkg_name = 'urdf_example'
#     file_subpath = 'description/bipednav_robot.urdf.xacro'


#     # Use xacro to process the file
#     xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
#     robot_description_raw = xacro.process_file(xacro_file).toxml()


#     # Configure the node
#     node_robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{'robot_description': robot_description_raw,
#         'use_sim_time': True}] # add other parameters here if required
#     )



#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
#         )


#     spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
#                     arguments=['-topic', 'robot_description',
#                                 '-entity', 'bipednav_robot'],
#                     output='screen')

#     # Teleop node
#     teleop_node = Node(
#         package='teleop_twist_keyboard',
#         executable='teleop_twist_keyboard',
#         prefix='xterm -e',  # Optional: Launches teleop in a new terminal window
#         output='screen'
#     )

#     # Run the node
#     return LaunchDescription([
#         gazebo,
#         node_robot_state_publisher,
#         spawn_entity,
#         teleop_node
#     ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'urdf_example'
    file_subpath = 'description/bipednav_robot.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'])
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'bipednav_robot'],
                        output='screen')

    delayed_spawn_entity = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[spawn_entity]
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        delayed_spawn_entity,
        teleop_node
    ])
