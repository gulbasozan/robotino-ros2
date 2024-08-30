#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from avatar_gui.avatar_gui import Ui_AvatarGUI, QtWidgets, QtCore
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QThread, pyqtSignal, QObject
import sys
import threading

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

PUSH_BUTTON_IDLE_STYLESHEET = """
            QPushButton {
                background-color: #6f6866; 
                color: #eff6ee; 
                border-radius: 16px;
            }
            QPushButton:pressed {
                background-color: #38302e    
            }
        
    """
PUSH_BUTTON_PRESSED_STYLESHEET = """
            QPushButton {
                background-color: #38302e; 
                color: #eff6ee; 
                border-radius: 16px;
            }
            QPushButton:pressed {
                background-color: #38302e    
            }
        
    """

# Qt signal needs to be defined under a QObject instance
# This is why it has a seperate class instead of
# having it under the ROS Node
class RosImageEmitter(QObject):
    image_signal = pyqtSignal(QImage) # Needs to be defined here specifically
    
    def __init__(self):
        super().__init__()

class AvatarGUINode(Node):
    def __init__(self, emitter):
        super().__init__("avatar_gui")
        
        self.safe_cmd_vel_pub_ = self.create_publisher(Twist, 'safe_cmd_vel', 10)
        self.safe_cmd_vel_msg_ = Twist()

        self.img_raw_sub_ = self.create_subscription(Image, 'img_raw', self.imgRawListenerCallback, 10)
        self.img_raw_msg_ = Image()

        self.bridge_ = CvBridge() # For image conversion

        self.emitter = emitter

        self.get_logger().info("AvatarGUINode initialized.")

    def imgRawListenerCallback(self, msg):
        try:
            cv_img = self.bridge_.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as error:
            self.get_logger().error(f"Could not convert the msg to cv2 img:\n {error}")
        
        # Need to format image to display it on QtApp
        height, width, channel = cv_img.shape
        bytes_per_line = 3 * width
        qt_image = QImage(cv_img.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()

        self.emitter.image_signal.emit(qt_image)

    def pubCallback(self, vel_x, vel_y, vel_omega):
        self.safe_cmd_vel_msg_.linear.x = vel_x 
        self.safe_cmd_vel_msg_.linear.y = vel_y 
        self.safe_cmd_vel_msg_.angular.z = vel_omega 

        self.safe_cmd_vel_pub_.publish(self.safe_cmd_vel_msg_)
        # self.get_logger().info("Publishing: '%s'" % self.safe_cmd_vel_msg_)


class KeyPressFilter(QtCore.QObject):
    def __init__(self, ros_node, gui):
        super().__init__()
        self.ros_node = ros_node
        self.gui = gui

    def eventFilter(self, obj, event):
        vel_x_ = 0.00
        vel_y_ = 0.00
        vel_omega_ = 0.00

        if event.type() == QtCore.QEvent.KeyPress:
            match event.key():
                case QtCore.Qt.Key_W:
                    vel_x_ = 0.05
                    self.gui.forward.setStyleSheet(PUSH_BUTTON_PRESSED_STYLESHEET)
                case QtCore.Qt.Key_A:
                    vel_y_ = 0.05
                    self.gui.left.setStyleSheet(PUSH_BUTTON_PRESSED_STYLESHEET)
                case QtCore.Qt.Key_S:
                    vel_x_ = -0.05
                    self.gui.backward.setStyleSheet(PUSH_BUTTON_PRESSED_STYLESHEET)
                case QtCore.Qt.Key_D:
                    vel_y_ = -0.05
                    self.gui.right.setStyleSheet(PUSH_BUTTON_PRESSED_STYLESHEET)
                case QtCore.Qt.Key_Q:
                    vel_omega_ = -0.05
                    self.gui.neg_angular.setStyleSheet(PUSH_BUTTON_PRESSED_STYLESHEET)
                case QtCore.Qt.Key_E:
                    vel_omega_ = 0.05
                    self.gui.pos_angular.setStyleSheet(PUSH_BUTTON_PRESSED_STYLESHEET)
                case QtCore.Qt.Key_Space:
                    vel_x_ = vel_y_ = vel_omega_ = 0.00
                    self.gui.halt.setStyleSheet(PUSH_BUTTON_PRESSED_STYLESHEET)

            self.ros_node.pubCallback(vel_x_, vel_y_, vel_omega_)

            return True

        if event.type() == QtCore.QEvent.KeyRelease:
            match event.key():
                case QtCore.Qt.Key_W:
                    self.gui.forward.setStyleSheet(PUSH_BUTTON_IDLE_STYLESHEET)
                case QtCore.Qt.Key_A:
                    self.gui.left.setStyleSheet(PUSH_BUTTON_IDLE_STYLESHEET)
                case QtCore.Qt.Key_S:
                    self.gui.backward.setStyleSheet(PUSH_BUTTON_IDLE_STYLESHEET)
                case QtCore.Qt.Key_D:
                    self.gui.right.setStyleSheet(PUSH_BUTTON_IDLE_STYLESHEET)
                case QtCore.Qt.Key_Q:
                    self.gui.neg_angular.setStyleSheet(PUSH_BUTTON_IDLE_STYLESHEET)
                case QtCore.Qt.Key_E:
                    self.gui.pos_angular.setStyleSheet(PUSH_BUTTON_IDLE_STYLESHEET)
                case QtCore.Qt.Key_Space:
                    self.gui.halt.setStyleSheet(PUSH_BUTTON_IDLE_STYLESHEET)
            return True

        return super().eventFilter(obj, event)

class AvatarGUIInstance(Ui_AvatarGUI):
    def __init__(self, ros_node, main_window):
        super().__init__()
        self.ros_node_ = ros_node
        self.setupUi(main_window)

        self.initButtonStyleSheet(PUSH_BUTTON_IDLE_STYLESHEET)
        self.initButtonSlotConnections(self.buttonCallback) # For non-keyboard clicks

        self.ros_node_.emitter.image_signal.connect(self.updateImage) # run updateImage when a signal emitted

    def buttonCallback(self, key):
        vel_x_ = 0.00
        vel_y_ = 0.00
        vel_omega_ = 0.00

        match key:
            case QtCore.Qt.Key_W:
                vel_x_ = 0.05
            case QtCore.Qt.Key_A:
                vel_y_ = 0.05
            case QtCore.Qt.Key_S:
                vel_x_ = -0.05
            case QtCore.Qt.Key_D:
                vel_y_ = -0.05
            case QtCore.Qt.Key_Q:
                vel_omega_ = -0.05
            case QtCore.Qt.Key_E:
                vel_omega_ = 0.05
            case QtCore.Qt.Key_Space:
                vel_x_ = vel_y_ = vel_omega_ = 0.00

        self.ros_node_.pubCallback(vel_x_, vel_y_, vel_omega_)

    def initButtonSlotConnections(self, callback):
        self.forward.clicked.connect(lambda: callback(QtCore.Qt.Key_W))
        self.backward.clicked.connect(lambda: callback(QtCore.Qt.Key_S))
        self.right.clicked.connect(lambda: callback(QtCore.Qt.Key_D))
        self.left.clicked.connect(lambda: callback(QtCore.Qt.Key_A))
        self.pos_angular.clicked.connect(lambda: callback(QtCore.Qt.Key_E))
        self.neg_angular.clicked.connect(lambda: callback(QtCore.Qt.Key_Q))
        self.halt.clicked.connect(lambda: callback(QtCore.Qt.Key_Space))

    def initButtonStyleSheet(self, styleSheet):
        self.forward.setStyleSheet(styleSheet)
        self.backward.setStyleSheet(styleSheet)
        self.right.setStyleSheet(styleSheet)
        self.left.setStyleSheet(styleSheet)
        self.pos_angular.setStyleSheet(styleSheet)
        self.neg_angular.setStyleSheet(styleSheet)
        self.halt.setStyleSheet(styleSheet)

    def updateImage(self, qt_image):
        pixmap = QPixmap.fromImage(qt_image)
        self.camera_label.setPixmap(pixmap)

# Run ROS as a Qt thread bc Qt signal
class ROSThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    # Ovveride defult QThread run function
    def run(self):
        rclpy.spin(self.node)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # Init Image Signal Emitter
    emitter = RosImageEmitter()

    # Init ROS Node
    node = AvatarGUINode(emitter)
    ros_thread = ROSThread(node)
    ros_thread.start()

    # Init QtApp
    app = QtWidgets.QApplication(sys.argv)
    main_window = QtWidgets.QMainWindow() 
    AvatarGUI = AvatarGUIInstance(node, main_window)

    # Init event filter
    key_press_filter = KeyPressFilter(node, AvatarGUI)
    app.installEventFilter(key_press_filter)

    # Run QtApp
    main_window.show()
    app.exec_()

    # Stop ROS Node
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == "__main__":
    main()
   