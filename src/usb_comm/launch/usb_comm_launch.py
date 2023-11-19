import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    config = os.path.join(
      get_package_share_directory('usb_comm'),
      'config',
      'usb_comm_config.yaml'
      )
    
    return LaunchDescription([
        Node(
            package="usb_comm",
            executable="usb_comm_node",
            name="usb_comm",
            namespace="usb_comm",
            output="screen",
            emulate_tty=True,
            parameters=[config]
        )
    ])
