from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
      Node(
         package='rgbc',
         executable='rgbc_node',
         name='rgbc_node',
         # python_executable='/home/anaconda3/envs/isaac/bin/python3',
      ),
   ])