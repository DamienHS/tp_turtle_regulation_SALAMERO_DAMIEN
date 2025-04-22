#!/usr/bin/env python3
"""
Noeud ROS 2 : turtle_regulator.py
Régulation en cap et en distance d'une tortue turtlesim vers un waypoint.
Implémente :
  - Partie 1 : calcul angle désiré, erreur e, commande angulaire u = Kp * e
  - Partie 2 : calcul distance euclidienne, erreur linéaire e1, commande linéaire v = Kp1 * e1
    • seuil distance_tolerance pour stopper la commande
    • publication d'un flag is_moving (Bool)
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleRegulator(Node):
    def __init__(self):
        super().__init__('turtle_regulator')

        # Paramètres
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Kp1', 0.5)
        self.declare_parameter('distance_tolerance', 0.1)

        # Etat interne
        self.turtle_pose = None
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

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.moving_pub = self.create_publisher(Bool,  'is_moving',    10)

    def cb_waypoint(self, msg: Float32MultiArray):
        self.waypoint = (msg.data[0], msg.data[1])
        self.get_logger().info(
            f"Waypoint reçu → x={self.waypoint[0]:.2f}, y={self.waypoint[1]:.2f}"
        )

    def cb_pose(self, msg: Pose):
        # Mise à jour de la pose
        self.turtle_pose = msg

        xA, yA = msg.x, msg.y
        xB, yB = self.waypoint

        # --- Partie 1 : régulation en cap ---
        # angle désiré
        desired_theta = math.atan2(yB - yA, xB - xA)
        # erreur e par arctan(tan((θ_desired - θ)/2))
        raw = (desired_theta - msg.theta) / 2.0
        e = math.atan(math.tan(raw))
        Kp = self.get_parameter('Kp').get_parameter_value().double_value
        u = Kp * e

        # --- Partie 2 : régulation en distance ---
        # distance euclidienne
        dist = math.hypot(xB - xA, yB - yA)
        e1 = dist
        Kp1 = self.get_parameter('Kp1').get_parameter_value().double_value
        v = Kp1 * e1
        tol = self.get_parameter('distance_tolerance').get_parameter_value().double_value

        # Flag déplacement
        moving = Bool()
        if e1 < tol:
            moving.data = False
            # ne plus publier sur cmd_vel
            self.cmd_pub.publish(Twist())  # arrêt brutal si nécessaire
            self.moving_pub.publish(moving)
            self.get_logger().info(f"Distance {e1:.2f} < tol {tol:.2f} → arrêt")
            return
        else:
            moving.data = True
            self.moving_pub.publish(moving)

        # Préparation de la commande Twist
        cmd = Twist()
        cmd.angular.z = u
        cmd.linear.x  = v
        self.cmd_pub.publish(cmd)

        # Logs
        self.get_logger().info(
            f"e_cap={e:.2f} rad, u_ang={u:.2f} | e_dist={e1:.2f}, v_lin={v:.2f}"
        )

    def _angle_diff(self, a: float, b: float) -> float:
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

