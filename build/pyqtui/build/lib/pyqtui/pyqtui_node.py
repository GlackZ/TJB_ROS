from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image

from PyQt5.QtCore import QObject, pyqtSignal
from cv_bridge import CvBridge
# import open3d 
# from sensor_msgs_py import point_cloud2
# import numpy as np

class PyqtuiNode(Node, QObject):
    signal_recv_img = pyqtSignal()  # 自定义的Qt信号，用于在ROS和Qt之间传递消息
    signal_recv_pos = pyqtSignal()

    def __init__(self):
        Node.__init__(self, "pyqtui_node")
        QObject.__init__(self)  # 初始化Qt对象
        self.get_logger().info('Pyqtui Node Started')
        self.bridge = CvBridge()
        self.cv_image = None
        self.pos_msg = None
        self.direction = 'None'
        # 订阅位置数据
        self.pos_subscription = self.create_subscription(String, 'position_data', self.pos_callback, 10)

        # 订阅机器人传回图像数据（单片机）
        self.img_subscription = self.create_subscription(Image, 'rgbc_node', self.img_callback, 10)

        # 发布机器人控制指令（单片机）
        self.publisher = self.create_publisher(String, 'robot_control', 10)
        self.timer = self.create_timer(0.5, self.robot_control_callback)

    def pos_callback(self, msg):
        self.pos_msg = msg.data
        self.signal_recv_pos.emit()


    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.signal_recv_img.emit()  # 发送Qt信号，通知Qt界面更新
        

    def robot_control_callback(self):
        msg = String()
        msg.data = self.direction
        self.get_logger().info(f"Robot control: {self.direction}")
        self.publisher.publish(msg)


    # def shutdown(self):
    #     self.shutdown_flag = True

    # @property
    # def shutdown_flag(self):
    #     return self._shutdown_flag


# def main(args=None):
#     rclpy.init(args=args)
#     pyqtui_node = PyqtuiNode()
#     rclpy.spin(pyqtui_node)
#     pyqtui_node.destroy_node()
#     rclpy.shutdown()
