import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class CorridorNode(Node):
    def __init__(self):
        super().__init__('corridor_node')
        
        # Publisher pour faire bouger le robot
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber pour lire le LIDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        self.get_logger().info("Mode Couloir activé : Je suis aveugle, j'utilise la Force (le LIDAR) !")

    def scan_callback(self, msg):
        # Le LIDAR du TurtleBot envoie souvent 360 valeurs.
        # Index 0 = Devant, 90 = Gauche, 180 = Derrière, 270 = Droite.
        
        # Fonction interne pour calculer la moyenne sur un petit angle (pour éviter le bruit du capteur)
        def get_distance(angle_center, angle_range=10):
            ranges = []
            for i in range(angle_center - angle_range, angle_center + angle_range):
                idx = i % len(msg.ranges)
                val = msg.ranges[idx]
                # On filtre les valeurs infinies ou trop proches (bruit du capteur)
                if 0.1 < val < 10.0:
                    ranges.append(val)
            if len(ranges) > 0:
                return sum(ranges) / len(ranges)
            return float('inf')

        # On récupère les distances clés
        dist_gauche = get_distance(90)
        dist_droite = get_distance(270)
        dist_face = get_distance(0, 15) # On regarde un peu plus large devant par sécurité

        cmd = Twist()

        # Vitesse d'avancement (Très lente comme tu l'as demandé)
        vitesse_base = 0.05

        # 1. SÉCURITÉ : S'il y a un mur juste en face, on pivote sur place
        if dist_face < 0.25:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.get_logger().warn("Mur droit devant ! Je pivote.")
            
        # 2. NAVIGATION : Suivi des murs
        else:
            cmd.linear.x = vitesse_base

            # Si on capte bien les murs de chaque côté
            if dist_gauche != float('inf') and dist_droite != float('inf'):
                
                # Calcul de l'erreur de centrage
                # Si c'est positif -> Il y a plus de place à gauche (On doit tourner à gauche)
                # Si c'est négatif -> Il y a plus de place à droite (On doit tourner à droite)
                erreur = dist_gauche - dist_droite

                # Kp (Coefficient Proportionnel) : Détermine la "nervosité" du volant
                kp = 0.8 
                
                # La rotation est proportionnelle à l'erreur
                cmd.angular.z = kp * erreur
                
            else:
                # Si on perd les murs, on continue tout droit par précaution
                cmd.angular.z = 0.0

        # On envoie l'ordre aux roues
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CorridorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # On arrête le robot en coupant le noeud
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()