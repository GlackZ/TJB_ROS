import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import cv2
import random

class PCLTestNode(Node):
    def __init__(self):
        super().__init__('pcl_test_node')
        self.publisher = self.create_publisher(PointCloud2, 'pointcloud', 10)
        self.timer = self.create_timer(0.5, self.callback)
        # self.cv_bridge = CvBridge()

        self.points = []
        for _ in range(100):
            x = random.uniform(-1, 1)
            y = random.uniform(-1, 1)
            z = random.uniform(-1, 1)
            self.points.append([x, y, z])

    def callback(self):
        msg = PointCloud2()
        points = self.points

        # 打包点云数据到pointcloud2消息
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'pointcloud'

        dtype = PointField.FLOAT32
        fields = [PointField(name='x', offset=0, datatype=dtype, count=1),
              PointField(name='y', offset=4, datatype=dtype, count=1),
              PointField(name='z', offset=8, datatype=dtype, count=1)]
        msg = point_cloud2.create_cloud(header, fields, points)

        # 发布点云数据
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pcl_test_node = PCLTestNode()
    rclpy.spin(pcl_test_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()