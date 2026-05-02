# Nom du fichier : calibrate_rotation.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
import math
import sys
import time

class RotationCalibrator(Node):
    def __init__(self):
        super().__init__('rotation_calibrator')
        
        # Publisher pour envoyer les commandes de rotation
        self.cmd_pub = self.create_publisher(Point32, '/pico/cmd_simple', 10)
        
        # Subscriber pour lire l'odométrie finale
        self.odom_sub = self.create_subscription(Point32, '/pico/odom_simple', self.odom_callback, 10)

        self.current_theta = 0.0
        self.total_rotation_rad = 0.0
        self.start_theta = None
        self.last_theta = 0.0

    def odom_callback(self, msg):
        # Gère le "saut" de l'angle de +PI à -PI et vice-versa
        if self.start_theta is None:
            self.start_theta = msg.z
            self.last_theta = msg.z

        angle_diff = msg.z - self.last_theta
        if angle_diff > math.pi:  # Saut de -PI à +PI
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi: # Saut de +PI à -PI
            angle_diff += 2 * math.pi
        
        self.total_rotation_rad += angle_diff
        self.last_theta = msg.z
        
        # Affiche la rotation en tours
        tours = self.total_rotation_rad / (2 * math.pi)
        sys.stdout.write(f"\rRotation totale : {self.total_rotation_rad:.2f} rad  ({tours:.2f} tours)")
        sys.stdout.flush()

    def get_final_rotation(self):
        return self.total_rotation_rad

    def stop_robot(self):
        try:
            self.cmd_pub.publish(Point32())
            time.sleep(0.1)
            self.cmd_pub.publish(Point32())
        except:
            pass

def main(args=None):
    print("\n--- SCRIPT DE CALIBRATION DE LA ROTATION (R_ROBOT) ---")
    print("1. Assurez-vous que le Pico est en MODE NORMAL (CALIBRATION_MODE = 0).")
    print("2. Assurez-vous que les constantes C1, C2, C3 sont déjà calibrées.")
    print("3. Placez le robot sur une surface plane.")
    print("4. Remettez l'odométrie à zéro :")
    print("   ros2 topic pub --once /pico/reset_odom std_msgs/msg/Empty")
    
    rotation_speed_rpm = 50.0
    print(f"\nLe robot va tourner sur lui-même à {rotation_speed_rpm} RPM.")
    print("Laissez-le faire plusieurs tours (ex: 5 ou 10) puis appuyez sur Ctrl+C.")
    print("-" * 60)

    rclpy.init(args=args)
    node = RotationCalibrator()
    
    # Commande pour faire tourner tous les moteurs à la même vitesse
    move_msg = Point32()
    move_msg.x = rotation_speed_rpm  # Moteur avant
    move_msg.y = rotation_speed_rpm  # Moteur gauche
    move_msg.z = rotation_speed_rpm  # Moteur droit

    final_rotation = 0.0
    try:
        # Attente pour que la souscription s'établisse
        time.sleep(1)
        while rclpy.ok():
            node.cmd_pub.publish(move_msg)
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.stop_robot()
        final_rotation = node.get_final_rotation()
        print("\n\n" + "-" * 60)
        print("--- ARRÊT DE LA MESURE ---")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if abs(final_rotation) > 0.1:
        print(f"Angle total mesuré par l'odométrie : {final_rotation:.4f} radians")
        try:
            val_entree = input("Entrez le nombre de tours COMPLETS que vous avez observés : ")
            tours_reels = float(val_entree)
            angle_theorique = tours_reels * 2 * math.pi * (1 if final_rotation > 0 else -1)

            # Lire la valeur actuelle de R_ROBOT dans odom.cpp pour l'afficher
            # Tu devras la rentrer manuellement ici pour le calcul.
            r_robot_actuel_str = input("Entrez la valeur actuelle de R_ROBOT dans odom.cpp : ")
            r_robot_actuel = float(r_robot_actuel_str)

            if abs(angle_theorique) > 0:
                r_robot_nouveau = r_robot_actuel * (final_rotation / angle_theorique)
                print("\n--- RÉSULTAT DE CALIBRATION ---")
                print(f"Angle théorique pour {tours_reels} tours : {angle_theorique:.4f} rad")
                print(f"\nCopiez cette nouvelle valeur dans votre fichier 'src/odom.cpp':")
                print(f"const float R_ROBOT = {r_robot_nouveau:.6f}f;")
                print("-" * 60)
            else:
                print("Le nombre de tours ne peut pas être zéro.")

        except (ValueError, ZeroDivisionError):
            print("\nErreur : Entrée non valide.")
    else:
        print("\nErreur : Rotation non détectée ou trop faible.")

if __name__ == '__main__':
    main()
