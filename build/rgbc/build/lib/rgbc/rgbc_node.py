import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RgbcNode(Node):
    def __init__(self):
        super().__init__('rgbc_node')
        self.get_logger().info('Rgbc Node Started')
        self.bridge = CvBridge()
        self.cv_image = None
        # self.device_path = '/dev/video0'

        # 发布图像数据（单片机）
        self.publisher = self.create_publisher(Image, 'rgbc_node', 10)
        self.timer = self.create_timer(0.01, self.publish_callback)
        # 打开摄像头
        # gst_pipeline = f"v4l2src device={self.device_path} ! videoconvert ! appsink"
        # self.rgb_cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        self.rgb_cap = cv2.VideoCapture(1, cv2.CAP_V4L)
        if not self.rgb_cap.isOpened():
            self.get_logger().error(f"Error: Could not open video device 1")
        self.rgb_cap.set(cv2.CAP_PROP_BRIGHTNESS, 10000)  # 修改这里以调整亮度

    def publish_callback(self):
        # msg = Image()
        # rgb_cap = cv2.VideoCapture(0, cv2.CAP_V4L)
        # print ('read')
        ret, frame = self.rgb_cap.read()
        if ret:
            # print('Frame')
            frame = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)
        # else:
            # pass
            # print('No frame')

def main(args=None):
    rclpy.init(args=args)
    rgbc_node = RgbcNode()
    rclpy.spin(rgbc_node)
    rgbc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()