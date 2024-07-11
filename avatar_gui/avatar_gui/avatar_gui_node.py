#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from avatar_gui.avatar_gui import Ui_AvatarGUI, QtWidgets, QtCore
from PyQt5.Qt import Qt
import sys
import threading

from geometry_msgs.msg import Twist


class AvatarGUINode(Node):
    def __init__(self):
        super().__init__("avatar_gui")
        
        self.safe_cmd_vel_pub_ = self.create_publisher(Twist, 'rto3/safe_cmd_vel', 10)
        self.safe_cmd_vel_msg_ = Twist()

        self.get_logger().info("AvatarGUINode initialized.")

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
        if event.type() == QtCore.QEvent.KeyPress:
            match event.key():
                case QtCore.Qt.Key_W:
                    print("forward Pressed")
                    self.ros_node.pubCallback(0.05, 0.00, 0.00)
                    self.gui.forward.setStyleSheet("QPushButton {background-color: black}")
                case QtCore.Qt.Key_A:
                    print("left Pressed")
                case QtCore.Qt.Key_S:
                    print("backward Pressed")
                case QtCore.Qt.Key_D:
                    print("right Pressed")
            return True

        if event.type() == QtCore.QEvent.KeyRelease:
            match event.key():
                case QtCore.Qt.Key_W:
                    self.gui.forward.setStyleSheet("QPushButton {background-color: white}")
                case QtCore.Qt.Key_A:
                    print("left Released")
                case QtCore.Qt.Key_S:
                    print("backward Released")
                case QtCore.Qt.Key_D:
                    print("right Released")
            return True

        return super().eventFilter(obj, event)

class AvatarGUIInstance(Ui_AvatarGUI):
    def __init__(self, ros_node, main_window):
        super().__init__()
        self.ros_node_ = ros_node
        self.setupUi(main_window)

    def main(self):
        self.forward.clicked.connect(self.buttonCallback)

    def buttonCallback(self):
        print("forward buttton clicked\n")
        self.ros_node_.pubCallback(0.00, 0.05, 0.00)

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = AvatarGUINode()

    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,))
    spin_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    main_window = QtWidgets.QMainWindow() 
    AvatarGUI = AvatarGUIInstance(node, main_window)
    AvatarGUI.main()

    key_press_filter = KeyPressFilter(node, AvatarGUI)
    app.installEventFilter(key_press_filter)

    main_window.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
   