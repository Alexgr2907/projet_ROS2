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
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(Float64, '/vision/direction_erreur', self.error_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- PARAMÈTRES DU CORRECTEUR PD ---
        self.vitesse_max = 0.12  # Vitesse de croisière
        
        # Kp : La force pour revenir au centre
        self.kp = 0.004 
        
        # Kd : L'amortisseur (LE SECRET DE LA FLUIDITÉ)
        # S'il oscille (gauche/droite), on augmente le Kd.
        self.kd = 0.010 
        
        # Mémoire pour le calcul de la dérivée
        self.erreur_precedente = 0.0

        # --- FREIN D'URGENCE (Pour l'angle droit plus tard) ---
        self.seuil_urgence = 170.0
        self.kp_pivot = 0.008
        self.en_urgence = False

    def error_callback(self, msg):
        erreur = msg.data
        cmd = Twist()

        # 1. Calcul de la dérivée (Vitesse de variation de l'erreur)
        derivee = erreur - self.erreur_precedente
        
        # 2. On sauvegarde l'erreur pour le prochain tour
        self.erreur_precedente = erreur

        # --- MACHINE A ETATS ---
        
        # Déclenchement de l'urgence si on perd totalement la trajectoire
        if not self.en_urgence and abs(erreur) > self.seuil_urgence:
            self.en_urgence = True
            self.get_logger().warn(f"ANGLE DROIT DÉTECTÉ (Erreur: {erreur:.0f})")

        # Fin de l'urgence quand on est réaligné
        elif self.en_urgence and abs(erreur) < 40.0:
            self.en_urgence = False
            self.get_logger().info("RÉALIGNEMENT TERMINÉ")

        # --- COMMANDES MOTEUR ---
        
        if self.en_urgence:
            # Mode "Stop & Tourne"
            cmd.linear.x = 0.0
            cmd.angular.z = -(self.kp_pivot * erreur)
        else:
            # Mode "Conduite Parfaite (PD)"
            # Le robot ralentit légèrement s'il y a de l'erreur (pour mieux prendre la courbe)
            # Formule magique : plus l'erreur est grande, plus on réduit la vitesse linéaire
            ralentissement = abs(erreur) * 0.0005 
            cmd.linear.x = max(0.05, self.vitesse_max - ralentissement)
            
            # Le pilotage au millimètre : Proportionnel + Dérivé
            cmd.angular.z = -(self.kp * erreur + self.kd * derivee)

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