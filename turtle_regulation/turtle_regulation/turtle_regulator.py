#!/usr/bin/env python3
"""
Noeud ROS 2 : turtle_regulator.py
Régulation en cap d'une tortue turtlesim vers un waypoint.
Implémente les points 4 et 5 du TP :
  - Calcul de l'angle désiré θ_desired = atan2(yB - yA, xB - xA)
  - Calcul de l'erreur e = atan( tan((θ_desired - θ) / 2) )
  - Commande proportionnelle u = Kp * e
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleRegulator(Node):
    def __init__(self):
        super().__init__('turtle_regulator')

        # Paramètre proportionnel
        self.declare_parameter('Kp', 1.0)

        # Etat interne
        self.turtle_pose = None
        # waypoint initialisé à (7.0, 7.0)
        self.waypoint = (7.0, 7.0)

        # Souscriptions
        self.create_subscription(
            Float32MultiArray,
            'way_point',
            self.cb_waypoint,
            10
        )
        self.create_subscription(
            Pose,
            'turtle1/pose',
            self.cb_pose,
            10
        )

        # Publisher de la commande de vitesse
        self.cmd_pub = self.create_publisher(
            Twist,
            'turtle1/cmd_vel',
            10
        )

    def cb_waypoint(self, msg: Float32MultiArray):
        """Met à jour le waypoint depuis le publisher set_way_point.py"""
        self.waypoint = (msg.data[0], msg.data[1])
        self.get_logger().info(
            f"Waypoint reçu → x={self.waypoint[0]:.2f}, y={self.waypoint[1]:.2f}"
        )

    def cb_pose(self, msg: Pose):
        """Met à jour la pose de la tortue et calcule la commande en cap"""
        # Mise à jour de la pose
        self.turtle_pose = msg
        self.get_logger().info(
            f"Pose tortue → x={msg.x:.2f}, y={msg.y:.2f}, θ={msg.theta:.2f}"
        )

        # Extraction des coordonnées
        xA, yA = msg.x, msg.y
        xB, yB = self.waypoint

        # 1) Calcul de l'angle désiré θ_desired
        desired_theta = math.atan2(yB - yA, xB - xA)
        self.get_logger().info(
            f"Angle désiré : θ_desired = {desired_theta:.2f} rad"
        )

        # 2) Calcul de l'erreur e = atan(tan((θ_desired - θ) / 2))
        raw = (desired_theta - msg.theta) / 2.0
        e = math.atan(math.tan(raw))
        self.get_logger().info(f"Erreur e = {e:.2f} rad")

        # 3) Commande proportionnelle u = Kp * e
        Kp = self.get_parameter('Kp').get_parameter_value().double_value
        u = Kp * e

        # Préparation du message Twist
        cmd = Twist()
        cmd.angular.z = u
        # Optionnel : avancer linéairement si aligné
        # epsilon = 0.05
        # cmd.linear.x = 1.0 if abs(e) <= epsilon else 0.0
        cmd.linear.x = 0.0

        # Publication de la commande
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f"Commande publiée → linear.x = {cmd.linear.x:.2f}, angular.z = {cmd.angular.z:.2f}"
        )

    def _angle_diff(self, a: float, b: float) -> float:
        """
        Ancienne méthode de normalisation, conservée si besoin.
        """
        diff = a - b
        return math.atan2(math.sin(diff), math.cos(diff))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleRegulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

