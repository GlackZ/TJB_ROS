from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
      Node(
         package='position',
         executable='position_node',
         name='position_node',
         # python_executable='/home/anaconda3/envs/isaac/bin/python3',
      ),
   ])