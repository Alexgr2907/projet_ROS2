import rclpy
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
        self.vitesse_lineaire = 0.1  # Vitesse d'avancement (m/s)
        self.kp = 0.008          # Coefficient proportionnel (force avec laquelle il tourne)

    def error_callback(self, msg):
        erreur = msg.data
        cmd = Twist()

        cmd.linear.x = self.vitesse_lineaire

        # Si la cible est à droite (erreur positive), il faut une vitesse angulaire négative pour tourner à droite.
        cmd.angular.z =  -(self.kp * erreur)

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