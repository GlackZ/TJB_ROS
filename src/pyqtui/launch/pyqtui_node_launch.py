from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
      Node(
         package='pyqtui',
         executable='pyqtui_node',
         name='pyqtui_node',
         # python_executable='/home/anaconda3/envs/isaac/bin/python3',
      ),
   ])