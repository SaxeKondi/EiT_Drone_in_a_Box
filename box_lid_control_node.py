#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from eit_box.box_controller import *
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Box_Node(Node):
    def __init__(self):
        super().__init__('box_node')

        # Create a publisher for String messages
        self.pub_current_lid_state = self.create_publisher(String, 'lid_state', 3)
        
        # Create a timer to publish messages periodically
        self.timer = self.create_timer(1.0, self.publish_lid_state)  # Publish every 1 second


        # subscribers
        self.sub_desired_lid_pos = self.create_subscription(
            Bool,                             # message type
            '/box_bool',                       # topic to subscribe to
            self.on_desired_lid_state_msg,    # callback function
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Lock to ensure exclusive access to the callback
        self.lock = threading.Lock()

        self.Box_control = LidController()

        self.get_logger().info("box_node Started...")


    def publish_lid_state(self):
        msg = String()
        msg.data = self.Box_control.current_state
        self.pub_current_lid_state.publish(msg)


    def on_desired_lid_state_msg(self, msg):
        # Attempt to acquire the lock
        if self.lock.locked():
            self.get_logger().warn("Callback already running; skipping this message.")
            return
        
        with self.lock:
            desired_lid_state_open = msg.data
            if self.Box_control.lid_open != desired_lid_state_open:
                if desired_lid_state_open:
                    self.Box_control.move_lid_tandem("open")
                else:
                    self.Box_control.move_lid_tandem("closed")
 

def main(args=None):
    rclpy.init(args=args)
    ros_node = Box_Node()
    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
