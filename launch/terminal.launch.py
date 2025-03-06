import os
os.environ["QT_QPA_PLATFORM"] = "xcb"


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# import descriptions
from launch_ros.actions import Node


def generate_launch_description():

    # use robot state publisher
    terminal_node = Node(
        package='control_package',
        executable='terminal',
        output='screen',

    )
    # empty launch_des
    launch_description = LaunchDescription()

    # gazebo launch
    launch_description.add_action(terminal_node)
    return launch_description
