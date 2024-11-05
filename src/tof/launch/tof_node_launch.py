from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()


    k4a_node = Node(
        package="tof",
        executable="publisher",
        name="publisher",
        parameters=[{"device": "/dev/tof_USB"}],
        output="screen",
        emulate_tty=True
    )



    ld.add_action(k4a_node)
    return ld
