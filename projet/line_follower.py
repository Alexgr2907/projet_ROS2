import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        # Outil pour convertir les images ROS 2 en images OpenCV
        self.bridge = CvBridge()
        
        # Souscription à la caméra (à vérifier si c'est le bon topic dans ton Gazebo)
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10)
            
        # Publication des commandes de vitesse
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Noeud de suivi de ligne démarré !')

    def image_callback(self, msg):
        # 1. Convertir le message ROS en image OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Récupérer les dimensions de l'image
        h, w, d = cv_image.shape
        
        # (Optionnel mais recommandé) Couper le haut de l'image pour ne regarder que le sol
        # cv_image = cv_image[int(h/2):h, 0:w] 
        # h = h - int(h/2)

        # 2. Traitement d'image : Convertir en HSV (meilleur pour détecter les couleurs)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # TODO : Définir les bornes de la couleur de la ligne à suivre (ici un exemple générique)
        # Il faudra ajuster ces valeurs selon la couleur (rouge, bleu...) dans Gazebo
        lower_color = np.array([0, 50, 50])
        upper_color = np.array([10, 255, 255])
        
        # Créer un masque : pixels dans la plage de couleur = blanc, le reste = noir
        mask = cv2.inRange(hsv, lower_color, upper_color)
        
        # 3. Calculer les moments de l'image pour trouver le centre de la ligne
        M = cv2.moments(mask)
        
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        if M['m00'] > 0:
            # Calculer les coordonnées (cx, cy) du centre de la ligne
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            # Dessiner un cercle rouge sur le centre détecté (pour le debug)
            cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), -1)
            
            # 4. Contrôleur Proportionnel
            # L'erreur est la différence entre le centre de l'image et le centre de la ligne
            erreur = cx - (w / 2)
            
            # Avancer doucement
            cmd.linear.x = 0.1 
            # Tourner en fonction de l'erreur (le "-0.002" est le gain proportionnel, à ajuster !)
            cmd.angular.z = float(-0.002 * erreur)
            
        else:
            self.get_logger().warning("Ligne perdue !")

        # Envoyer la commande au robot
        self.cmd_pub.publish(cmd)
        
        # Afficher l'image pour voir ce que le robot voit (très utile pour débugger)
        cv2.imshow("Vue Caméra", cv_image)
        cv2.imshow("Masque", mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
