import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

import sys
sys.path.clear()
anaconda_path = ['/home/glack/Documents/project/Tjb/src/position/position',
 '/home/glack/Documents/project/Tjb/build/pyqtui',
 '/home/glack/Documents/project/Tjb/install/pyqtui/lib/python3.10/site-packages',
 '/home/glack/Documents/project/Tjb/build/position',
 '/home/glack/Documents/project/Tjb/install/position/lib/python3.10/site-packages',
 '/home/glack/Documents/project/Tjb/install/azure_kinect_ros2_driver/lib/python3.10/site-packages',
 '/opt/ros/humble/lib/python3.10/site-packages',
 '/opt/ros/humble/local/lib/python3.10/dist-packages',
 '/opt/ros/humble/lib',
 '/home/glack/anaconda3/envs/isaac/lib/python310.zip',
 '/home/glack/anaconda3/envs/isaac/lib/python3.10',
 '/home/glack/anaconda3/envs/isaac/lib/python3.10/lib-dynload',
 '/home/glack/anaconda3/envs/isaac/lib/python3.10/site-packages']

wrong_path = ['/home/glack/Documents/project/Tjb/install/position/lib/position',
 '/home/glack/Documents/project/Tjb/build/pyqtui',
 '/home/glack/Documents/project/Tjb/install/pyqtui/lib/python3.10/site-packages',
 '/home/glack/Documents/project/Tjb/build/position',
 '/home/glack/Documents/project/Tjb/install/position/lib/python3.10/site-packages',
 '/home/glack/Documents/project/Tjb/install/azure_kinect_ros2_driver/lib/python3.10/site-packages',
 '/opt/ros/humble/lib/python3.10/site-packages',
 '/opt/ros/humble/local/lib/python3.10/dist-packages',
 '/usr/lib/python310.zip',
 '/usr/lib/python3.10',
 '/usr/lib/python3.10/lib-dynload',
 '/usr/local/lib/python3.10/dist-packages',
 '/usr/lib/python3/dist-packages',
 '/home/glack/anaconda3/envs/isaac/lib/python310.zip',
 '/home/glack/anaconda3/envs/isaac/lib/python3.10',
 '/home/glack/anaconda3/envs/isaac/lib/python3.10/lib-dynload',
 '/home/anaconda3/envs/isaac/lib/python3.10/site-packages']
# anaconda_path=['/home/glack/anaconda3/envs/isaac/lib/python310.zip',
#                 '/home/glack/anaconda3/envs/isaac/lib/python3.10',
#                 '/home/glack/anaconda3/envs/isaac/lib/python3.10/lib-dynload',
#                  '/home/anaconda3/envs/isaac/lib/python3.10/site-packages']
for path in anaconda_path:
    sys.path.append(path)


from . import trans_to_point 
from . import icp

class PositionNode(Node):
    def __init__(self):
        super().__init__('position_node')
        self.get_logger().info('Position Node Started')
        # self.get_logger().info(sys.path)

        # 初始化点云数据
        self.points = []

        # 订阅深度图像数据
        # self.subscription = self.create_subscription(Image, '/k4a/depth_to_rgb/image_raw', self.subscribe_callback, 10)
        # 订阅点云数据
        self.subscription = self.create_subscription(PointCloud2, 'cloud', self.subscribe_callback, 10)


        # 发布位置数据到ui界面
        self.publisher = self.create_publisher(String, 'position_data', 10)
        self.timer = self.create_timer(0.5, self.position_data_publish_callback)

    def subscribe_callback(self, msg):
        # 通过深度图像数据生成点云数据
        # trans = trans_to_point.point_cloud_generator_depth(msg, focal_length=300, scalingfactor=1000)

        # 通过点云消息生成点云数据
        trans = trans_to_point.point_cloud_generator_pointcloudmsg(msg)

        self.points = trans.calculate()

    def position_data_publish_callback(self):
        msg = String()
        if len(self.points) == 0:
            msg.data = 'No data'
        else:
            msg.data = icp.get_position(self.points)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    position_node = PositionNode()
    rclpy.spin(position_node)
    position_node.destroy_node()
    rclpy.shutdown()







# a.show_point_cloud()
# df = a.df
# np.save('pc.npy',df)