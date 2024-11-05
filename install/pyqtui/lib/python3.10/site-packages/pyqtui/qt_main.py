import sys
import threading
import time
from . import mainwindow_ui
from . import pyqtui_node
import rclpy
import cv2
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap

class MainWindow(QMainWindow, mainwindow_ui.Ui_MainWindow):
    def __init__(self, ros_node):
        super(MainWindow, self).__init__()
        self.ui = mainwindow_ui.Ui_MainWindow()
        self.ui.setupUi(self)
        self.ros_node = ros_node
        ros_node.signal_recv_img.connect(self.update_image)
        ros_node.signal_recv_pos.connect(self.update_position)

    def update_image(self):
        if self.ros_node.cv_image is not None:
            height, width, channel = self.ros_node.cv_image.shape
            bytesPerLine = 3 * width
            qImg = QImage(self.ros_node.cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(qImg)
            scaled_pixmap = pixmap.scaled(self.ui.label_img.width(), self.ui.label_img.height(), Qt.KeepAspectRatio)
            self.ui.label_img.setPixmap(scaled_pixmap)

    def update_position(self):
        if self.ros_node.pos_msg is not None:
            self.ui.label_pos.setText(self.ros_node.pos_msg)

    def imgSave(self):
        if self.ros_node.cv_image is not None:
            cv2.imwrite('img.jpg', self.ros_node.cv_image)

    def robot_control(self, direction):
        self.ros_node.direction = direction

# 在单独线程中运行ROS节点的spin函数
def ros_spin(node):
    # while not node.is_shutdown:
    while True:
        rclpy.spin_once(node)
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    ros_node = pyqtui_node.PyqtuiNode()

    app = QApplication(sys.argv)
    main_window = MainWindow(ros_node)
    main_window.show()

    # 在单独线程中运行ROS节点的spin函数
    thread_spin = threading.Thread(target=ros_spin, args=(ros_node,))
    thread_spin.start()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()