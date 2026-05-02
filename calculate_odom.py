import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
import math
import sys

class OdomCalibrator(Node):
    def __init__(self):
        super().__init__('odom_calibrator')
        
        # Dictionnaire pour stocker les derniers comptes de ticks de chaque capteur
        self.ticks = {
            1: {'x': 0.0, 'y': 0.0},
            2: {'x': 0.0, 'y': 0.0},
            3: {'x': 0.0, 'y': 0.0}
        }

        # Création des abonnements aux topics des capteurs
        self.sub1 = self.create_subscription(Point32, '/pico/capteur1', lambda msg: self.sensor_callback(msg, 1), 10)
        self.sub2 = self.create_subscription(Point32, '/pico/capteur2', lambda msg: self.sensor_callback(msg, 2), 10)
        self.sub3 = self.create_subscription(Point32, '/pico/capteur3', lambda msg: self.sensor_callback(msg, 3), 10)

        self.total_ticks = {1: 0.0, 2: 0.0, 3: 0.0}
        self.timer = self.create_timer(0.1, self.print_ticks)

    def sensor_callback(self, msg, sensor_id):
        # Le Pico accumule déjà les ticks, on prend juste la dernière valeur
        self.ticks[sensor_id]['x'] = msg.x
        self.ticks[sensor_id]['y'] = msg.y

    def print_ticks(self):
        # Calcule la magnitude (distance totale) pour chaque capteur
        for i in range(1, 4):
            self.total_ticks[i] = math.sqrt(self.ticks[i]['x']**2 + self.ticks[i]['y']**2)

        # Affiche les comptes de ticks actuels sur une seule ligne
        sys.stdout.write("\r\033[K") 
        ticks_str = (f"Total Ticks | C1: {self.total_ticks[1]:.0f} | "
                     f"C2: {self.total_ticks[2]:.0f} | "
                     f"C3: {self.total_ticks[3]:.0f}")
        sys.stdout.write(ticks_str)
        sys.stdout.flush()

    def get_final_ticks(self):
        return self.total_ticks

def main(args=None):
    print("\n--- SCRIPT DE CALIBRATION MANUELLE (C1, C2, C3) ---")
    print("1. Assurez-vous que le Pico est en CALIBRATION_MODE = 1.")
    print("2. Placez le robot au début d'une règle sur une surface texturée.")
    print("3. Remettez les compteurs à zéro avec la commande :")
    print("   ros2 topic pub --once /pico/reset_odom std_msgs/msg/Empty")
    print("\n4. LANCEZ CE SCRIPT.")
    print("5. POUSSEZ MANUELLEMENT le robot en ligne droite sur une distance connue.")
    print("6. Appuyez sur Ctrl+C pour arrêter et calculer les constantes.")
    print("-" * 60)

    rclpy.init(args=args)
    node = OdomCalibrator()
    final_ticks = {}
 
    try:
        # Le script écoute passivement les topics des capteurs jusqu'à ce que
        # l'utilisateur appuie sur Ctrl+C.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # L'interruption clavier est capturée ici, ce qui arrête rclpy.spin().
        # On ne fait rien de spécial, on laisse juste le programme continuer.
        pass
 
    # --- Le script continue ici après que l'utilisateur ait fait Ctrl+C ---
 
    # 1. On récupère les dernières valeurs de ticks avant de tout couper.
    final_ticks = node.get_final_ticks()
    print("\n\n" + "-" * 60)
    print("--- ARRÊT DE LA MESURE ---")
 
    # 2. On nettoie proprement les ressources ROS.
    node.destroy_node()
    rclpy.shutdown()
 
    # 3. On effectue les calculs avec les valeurs récupérées.
    if any(t > 1 for t in final_ticks.values()):
        print(f"Ticks totaux (magnitude) :")
        print(f"  - Capteur 1: {final_ticks[1]:.0f}")
        print(f"  - Capteur 2: {final_ticks[2]:.0f}")
        print(f"  - Capteur 3: {final_ticks[3]:.0f}")
        
        try:
            val_entree = input("\nEntrez la distance réelle parcourue (en cm) : ")
            reelle_m = float(val_entree) / 100.0

            # Calcul des constantes
            c1 = reelle_m / final_ticks[1] if final_ticks[1] != 0 else 0
            c2 = reelle_m / final_ticks[2] if final_ticks[2] != 0 else 0
            c3 = reelle_m / final_ticks[3] if final_ticks[3] != 0 else 0

            print(f"\n--- RÉSULTATS DE CALIBRATION ---")
            print(f"Distance réelle : {reelle_m:.4f} m")
            print("\nCopiez ces valeurs dans votre fichier 'src/odom.cpp':")
            print(f"const float C1 = {c1:.8f}f;")
            print(f"const float C2 = {c2:.8f}f;")
            print(f"const float C3 = {c3:.8f}f;")
            print("-" * 60)

        except ValueError:
            print("\nErreur : Entrée non valide. Veuillez entrer un nombre.")
        except ZeroDivisionError:
            print("\nErreur : Division par zéro. Un des capteurs n'a pas renvoyé de ticks.")
    else:
        print("\nErreur : Mouvement non détecté ou trop faible. Aucun calcul effectué.")

if __name__ == '__main__':
    main()