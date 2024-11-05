import os
 
from ament_index_python.packages import get_package_share_directory
 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
 
def generate_launch_description():
   positon_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('position'), 'launch'),
         '/position_node_launch.py'])
      )
   pyqtui_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('pyqtui'), 'launch'),
         '/pyqtui_node_launch.py'])
      )
   rgbc_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('rgbc'), 'launch'),
         '/rgbc_node_launch.py'])
      )
   tof_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('tof'), 'launch'),
         '/tof_node_launch.py'])
      )

   return LaunchDescription([
      positon_node,
      pyqtui_node,
      rgbc_node,
      tof_node
   ])