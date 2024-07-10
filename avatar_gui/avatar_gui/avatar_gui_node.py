#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from avatar_gui.avatar_gui import Ui_AvatarGUI, QtWidgets
import sys
import threading

from geometry_msgs.msg import Twist


class AvatarGUINode(Node):
    def __init__(self):
        super().__init__("avatar_gui")
        
        self.safe_cmd_vel_pub_ = self.create_publisher(Twist, 'rto3/safe_cmd_vel', 10)
        # timer_period = 0.5
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("AvatarGUINode initialized.")

    def pub_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.angular.z = 0.0

        self.safe_cmd_vel_pub_.publish(msg)
        self.get_logger().info("Publishing: '%s'" % msg)

class AvatarGUIInstance(Ui_AvatarGUI):
    def __init__(self, ros_node, main_window):
        super().__init__()
        self.ros_node_ = ros_node
        self.setupUi(main_window)


    def main(self):
        print("forward button: ", self.forward)

        self.forward.clicked.connect(self.button_callback)

    def button_callback(self):
        print("forward buttton clicked\n")
        self.ros_node_.pub_callback()


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
    main_window.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
   