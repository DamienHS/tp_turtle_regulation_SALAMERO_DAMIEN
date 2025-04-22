#!/usr/bin/env python3
"""
Noeud ROS2 : set_way_point.py
Publie périodiquement un point de consigne (waypoint) en cap.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SetWayPoint(Node):
    def __init__(self):
        super().__init__('set_way_point')
        self.declare_parameter('waypoint_x', 7.0)
        self.declare_parameter('waypoint_y', 7.0)
        self.pub = self.create_publisher(Float32MultiArray, 'way_point', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoint)

    def publish_waypoint(self):
        x = self.get_parameter('waypoint_x').get_parameter_value().double_value
        y = self.get_parameter('waypoint_y').get_parameter_value().double_value
        msg = Float32MultiArray(data=[x, y])
        self.pub.publish(msg)
        self.get_logger().info(f"Waypoint published: x={x}, y={y}")

def main(args=None):
    rclpy.init(args=args)
    node = SetWayPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
