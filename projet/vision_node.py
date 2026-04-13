"""import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
# from rclpy.qos import qos_profile_sensor_data # probleme reception image

class VisionNodeSubscriber(Node):
    def __init__(self):
        super().__init__('vision_node_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image,'/image_raw',self.listener_callback,10)
        self.subscription  # to prevent unused variable warning
        self.error_publisher = self.create_publisher(Float64, '/vision/direction_erreur', 10)
        # self.declare_parameter('roundabout_direction', 'right')

        self.largeur_route = 300.0  # Valeur par défaut au démarrage
        self.derniere_erreur = 0.0  # Pour savoir de quel côté tourner si on perd TOUT

    def listener_callback(self, msg):
        try:
            # Conversion directe du message ROS 2 vers une image OpenCV (BGR)
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erreur cv_bridge : {e}")
            return

        if image is not None:
            height, width, _ = image.shape
    
            # On ne garde que la moitié inférieure de l'image (les pieds du robot)
            image_crop = image[int(height/3):height, 0:width]
            # La conversion en HSV
            hsv_image = cv2.cvtColor(image_crop, cv2.COLOR_BGR2HSV)

            # Seuils pour le Vert (Teinte entre 40 et 80 environ)
            lower_green = np.array([35, 20, 20])
            upper_green = np.array([85, 255, 255])

            # Création du masque binaire : les pixels verts deviennent blancs (255), le reste noir (0)
            mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

            # Première partie du rouge (les teintes basses : 0 à 10)
            lower_red1 = np.array([0, 50, 50])
            upper_red1 = np.array([10, 255, 255])
            mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)

            # Deuxième partie du rouge (les teintes hautes : 170 à 179)
            lower_red2 = np.array([170, 50, 50])
            upper_red2 = np.array([179, 255, 255])
            mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

            # Fusion des deux masques rouges avec un "OU" logique
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)

            # Initialisation des positions 
            cx_green = None
            cx_red = None

            M_green = cv2.moments(mask_green)
            if M_green['m00'] > 0:
                cx_green = int(M_green['m10'] / M_green['m00'])


            M_red = cv2.moments(mask_red)
            if M_red['m00'] > 0:
                cx_red = int(M_red['m10'] / M_red['m00'])

            centre_robot = width / 2  # Le nez du robot est exactement au milieu de l'image
            target_x = centre_robot # Par défaut, on vise tout droit

            # direction_rond_point = self.get_parameter('roundabout_direction').get_parameter_value().string_value

            if cx_green is not None and cx_red is not None:
                # Il voit tout : il vise le milieu et MET À JOUR sa mémoire de la route
                target_x = (cx_green + cx_red) / 2
                self.largeur_route = abs(cx_red - cx_green)
                
            elif cx_green is not None:
                # Il ne voit que le vert : il s'écarte exactement de la MOITIÉ de la route mémorisée
                offset = self.largeur_route / 2
                target_x = cx_green + offset
                
            elif cx_red is not None:
                # Il ne voit que le rouge
                offset = self.largeur_route / 2
                target_x = cx_red - offset
                
            else:
                # SÉCURITÉ ABSOLUE : Il ne voit PLUS RIEN.
                # On force une cible extrême du même côté que la dernière fois pour le faire pivoter
                target_x = centre_robot + (500 if self.derniere_erreur > 0 else -500)

            # Calcul final de l'erreur
            # Si target_x > centre_robot, l'erreur est positive (tourner à droite)
            # Si target_x < centre_robot, l'erreur est négative (tourner à gauche)
            erreur = target_x - centre_robot
            self.derniere_erreur = erreur # On sauvegarde pour le prochain tour

            # Publication de l'erreur 
            msg_erreur = Float64()
            msg_erreur.data = float(erreur)
            self.error_publisher.publish(msg_erreur)

            # cv2.imshow("Mon Masque Vert", mask_green)
            # cv2.imshow("Mon Masque Rouge", mask_red)

            # 1. Dessiner un point sur la cible pour voir où le robot veut aller
            # On dessine un cercle bleu sur l'image découpée en couleur
            cv2.circle(image_crop, (int(target_x), int(image_crop.shape[0]/2)), 10, (255, 0, 0), -1)
            
            # 2. Coller les deux masques côte à côte (Horizontal Stack)
            # np.hstack permet de fusionner deux images de même taille horizontalement
            masques_combines = np.hstack((mask_green, mask_red))
            
            # 3. Afficher les fenêtres
            cv2.imshow("Cerveau du robot : Vert | Rouge", masques_combines)
            cv2.imshow("Vue Camera (Cible en bleu)", image_crop)
            cv2.waitKey(1)
        

def main(args=None):
    rclpy.init(args=args)
    node = VisionNodeSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.error_publisher = self.create_publisher(Float64, '/vision/direction_erreur', 10)
        
        # Mémoire de la largeur de la route
        self.moitie_route = 200.0 

    def listener_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        h, w, _ = image.shape
        
        roi = image[int(h * 0.5):h, 0:w]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Saturation remontée à 50 pour éviter de confondre le sol beige avec du rouge/vert !
        mask_green = cv2.inRange(hsv, np.array([35, 50, 20]), np.array([85, 255, 255]))
        mask_red1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
        mask_red2 = cv2.inRange(hsv, np.array([170, 70, 50]), np.array([179, 255, 255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # 3. Calcul des centres
        cx_green = None
        cx_red = None

        M_green = cv2.moments(mask_green)
        if M_green['m00'] > 500: # Anti-bruit
            cx_green = int(M_green['m10'] / M_green['m00'])

        M_red = cv2.moments(mask_red)
        if M_red['m00'] > 500:
            cx_red = int(M_red['m10'] / M_red['m00'])

        centre_image = w / 2
        target_x = centre_image 

        # 4. LOGIQUE DE CIBLAGE
        if cx_green is not None and cx_red is not None:
            target_x = (cx_green + cx_red) / 2
            
            # On calcule la largeur actuelle
            largeur_mesuree = abs(cx_red - cx_green) / 2.0
            
            # SÉCURITÉ : On n'enregistre la largeur QUE si c'est une route normale (ex: max 250 pixels)
            # Si c'est plus grand, c'est qu'on arrive au rond point, on garde l'ancienne valeur !
            if largeur_mesuree < 300.0: 
                self.moitie_route = largeur_mesuree
            
        elif cx_green is not None:
            # Ligne verte uniquement
            target_x = cx_green + self.moitie_route
            
        elif cx_red is not None:
            # Ligne rouge uniquement
            target_x = cx_red - self.moitie_route

        # 5. Calcul et publication de l'erreur
        erreur = target_x - centre_image
        
        msg_erreur = Float64()
        msg_erreur.data = float(erreur)
        self.error_publisher.publish(msg_erreur)

        # 6. Affichages visuels
        cv2.circle(roi, (int(target_x), int(roi.shape[0]/2)), 8, (255, 0, 0), -1)
        cv2.imshow("Camera Robot", roi)
        
        # --- LA FENÊTRE DE DIAGNOSTIC DES COULEURS ---
        masques = np.hstack((mask_green, mask_red))
        cv2.imshow("Filtres Vert | Rouge", masques)
        
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()