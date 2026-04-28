"""import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.subscription = self.create_subscription(Float64,'/vision/direction_erreur',self.error_callback,10)
        # 2. On crée le publisher pour envoyer les vitesses aux roues du TurtleBot
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Paramètres de conduite (à ajuster lors de vos tests sur Gazebo)
        self.vitesse_lineaire = 0.12  # Vitesse d'avancement (m/s)
        self.kp = 0.008          # Coefficient proportionnel (force avec laquelle il tourne)

    def error_callback(self, msg):
        erreur = msg.data
        cmd = Twist()

        # ÉTAT 1 : RECHERCHE ET RECENTRAGE (Si l'erreur est très grande)
        if abs(erreur) > 150: 
            cmd.linear.x = 0.0  # On s'arrête d'avancer !
            # On pivote sur place. On multiplie par 1.5 pour tourner un peu plus vite
            cmd.angular.z = -(self.kp * erreur * 1.5) 
            
        # ÉTAT 2 : CONDUITE NORMALE
        else:
            cmd.linear.x = self.vitesse_lineaire
            cmd.angular.z = -(self.kp * erreur)

        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Sécurité : on envoie une commande de vitesse nulle avant de couper le nœud
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    """

# VERSION 2 
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(Float64, '/vision/direction_erreur', self.error_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Paramètres de conduite
        self.vitesse_max = 0.10 
        self.kp_normal = 0.005   
        self.kp_pivot = 0.008 
        
        # --- NOUVEAU : LE DOUBLE SEUIL ---
        self.seuil_declenchement = 120.0 # Erreur pour déclencher l'arrêt
        self.seuil_validation = 60.0     # Erreur requise pour avoir le droit de repartir
        
        # Mémoire d'état
        self.en_pivot = False 

    def error_callback(self, msg):
        erreur = msg.data
        cmd = Twist()

        # --- MACHINE À ÉTATS : DOUBLE SEUIL ---
        
        # 1. On vérifie s'il faut COMMENCER à pivoter
        if not self.en_pivot and abs(erreur) > self.seuil_declenchement:
            self.en_pivot = True
            self.get_logger().warn(f"🛑 DÉBUT DU PIVOT (Erreur: {erreur:.0f})")

        # 2. On vérifie s'il faut ARRÊTER de pivoter (il est redevenu bien droit)
        elif self.en_pivot and abs(erreur) < self.seuil_validation:
            self.en_pivot = False
            self.get_logger().info("✅ FIN DU PIVOT, robot réaligné ! On repart.")

        # --- APPLICATION DES VITESSES ---
        
        if self.en_pivot:
            # Il a l'ordre de pivoter, INTERDICTION d'avancer
            cmd.linear.x = 0.0
            
            # On tourne fort vers le centre. 
            # (On garde le signe - car si l'erreur est positive, il faut tourner en négatif)
            cmd.angular.z = -(self.kp_pivot * erreur)
            
        else:
            # Tout va bien, on avance normalement
            cmd.linear.x = self.vitesse_max
            cmd.angular.z = -(self.kp_normal * erreur)

        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.publisher.publish(Twist()) # Arrêt d'urgence à la coupure
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
#Version 3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        # --- LES 3 OREILLES DU ROBOT ---
        self.sub_vision = self.create_subscription(Float64, '/vision/direction_erreur', self.vision_callback, 10)
        self.sub_lidar = self.create_subscription(Float64, '/lidar/repulsion', self.lidar_callback, 10)
        self.sub_etat = self.create_subscription(Int32, '/etat_mission', self.etat_callback, 10)
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- PARAMÈTRES DU CORRECTEUR PD ---
        self.vitesse_max = 0.060  # Vitesse de croisière
        
        # Kp : La force pour revenir au centre
        self.kp = 0.0040
        self.kd = 0.015
        
        # --- MÉMOIRES D'ÉTAT ---
        self.erreur_vision = 0.0
        self.force_lidar = 0.0
        self.challenge_actuel = 1
        self.erreur_precedente = 0.0

        # --- FREIN D'URGENCE 
        self.seuil_urgence = 250.0
        self.kp_pivot = 0.008
        self.en_urgence = False

    def etat_callback(self, msg):
        """Met à jour le numéro du challenge actuel"""
        self.challenge_actuel = msg.data

    def lidar_callback(self, msg):
        """Récupère la force de répulsion de l'obstacle"""
        self.force_lidar = msg.data
    
    def vision_callback(self, msg):
        """C'est ici que le cerveau prend sa décision finale à chaque image"""
        self.erreur_vision = msg.data
        
        # ==========================================
        # FUSION DE CAPTEURS (VISION + LIDAR)
        # ==========================================
        if self.challenge_actuel >= 2:
            # Challenge 2 : Esquive ! On combine la ligne et l'obstacle
            erreur_totale = self.erreur_vision + self.force_lidar
        else:
            # Challenge 1 : On suit la ligne, on est aveugle aux obstacles
            erreur_totale = self.erreur_vision


        # Affichage de debug facultatif pour comprendre ce qui se passe
        # self.get_logger().info(f"CH:{self.challenge_actuel} | Cam:{self.erreur_vision:.0f} | Lid:{self.force_lidar:.0f} | Tot:{erreur_totale:.0f}")

        # 1. Calcul de la dérivée (Vitesse de variation de l'erreur totale)
        derivee = erreur_totale - self.erreur_precedente
        self.erreur_precedente = erreur_totale

        cmd = Twist()

        # --- MACHINE A ETATS ---
        
        # Déclenchement de l'urgence si on perd totalement la trajectoire ou esquive violente
        if not self.en_urgence and abs(erreur_totale) > self.seuil_urgence:
            self.en_urgence = True
            self.get_logger().warn(f"MOUVEMENT BRUSQUE DÉTECTÉ (Erreur: {erreur_totale:.0f})")

        # Fin de l'urgence quand on est réaligné
        elif self.en_urgence and abs(erreur_totale) < 40.0:
            self.en_urgence = False
            self.get_logger().info("RÉALIGNEMENT TERMINÉ")

        # --- COMMANDES MOTEUR ---
        
        if self.en_urgence:
            # Mode "Stop & Tourne"
            cmd.linear.x = 0.0
            cmd.angular.z = -(self.kp_pivot * erreur_totale)
        else:
            # Mode "Conduite Parfaite (PD)"
            # Le robot ralentit légèrement s'il y a de l'erreur (pour mieux prendre la courbe/esquive)
            ralentissement = abs(erreur_totale) * 0.0005 
            cmd.linear.x = max(0.05, self.vitesse_max - ralentissement)
            
            # Le pilotage au millimètre : Proportionnel + Dérivé
            cmd.angular.z = -(self.kp * erreur_totale + self.kd * derivee)

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.publisher.publish(Twist()) 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()