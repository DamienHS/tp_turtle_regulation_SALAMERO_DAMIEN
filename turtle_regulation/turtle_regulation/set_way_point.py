#!/usr/bin/env python3
"""
Noeud ROS 2 : set_way_point.py
Publie périodiquement un waypoint et propose un service pour le modifier à la volée.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from turtle_interfaces.srv import SetWayPoint


class SetWayPoint(Node):
    def __init__(self):
        super().__init__('set_way_point')

        # Coordonnées initiales du waypoint
        self.waypoint_x = 7.0
        self.waypoint_y = 7.0

        # Publisher du waypoint
        self.pub = self.create_publisher(
            Float32MultiArray,
            'way_point',
            10
        )

        # Timer pour publier à 1 Hz
        self.timer = self.create_timer(1.0, self.publish_waypoint)

        # Service pour mettre à jour le waypoint
        self.srv = self.create_service(
            SetWayPoint,
            'set_waypoint_service',
            self.cb_set_waypoint
        )

    def publish_waypoint(self):
        msg = Float32MultiArray(data=[self.waypoint_x, self.waypoint_y])
        self.pub.publish(msg)
        self.get_logger().info(
            f"Waypoint publié : x={self.waypoint_x:.2f}, y={self.waypoint_y:.2f}"
        )

    def cb_set_waypoint(self, request, response):
        # Met à jour les coordonnées du waypoint
        self.waypoint_x = request.x
        self.waypoint_y = request.y
        response.res = True
        self.get_logger().info(
            f"Waypoint mis à jour via service : x={self.waypoint_x:.2f}, y={self.waypoint_y:.2f}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SetWayPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

