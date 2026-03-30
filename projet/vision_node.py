import rclpy
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
            lower_green = np.array([40, 50, 50])
            upper_green = np.array([80, 255, 255])

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

            height, width, _ = image.shape
            centre_robot = width / 2  # Le nez du robot est exactement au milieu de l'image
            target_x = centre_robot # Par défaut, on vise tout droit

            # direction_rond_point = self.get_parameter('roundabout_direction').get_parameter_value().string_value

            # la direction ciblé au milieu des 2 lignes
            if cx_green is not None and cx_red is not None:
                target_x = (cx_green + cx_red) / 2
                self.get_logger().info(f"Largeur route : {cx_red - cx_green} pixels")

            # Seulement la ligne verte est visible
            elif cx_green is not None:
                # Si le robot n'a que la ligne gauche, il doit se décaler vers la droite d'une certaine distance
                offset = 150 # Distance arbitraire en pixels (à ajuster lors de vos tests)
                target_x = cx_green + offset
                
            # Seulement la ligne rouge est visible
            elif cx_red is not None:
                # Si le robot n'a que la ligne droite, il doit se décaler vers la gauche
                offset = 150
                target_x = cx_red - offset

            # Calcul final de l'erreur
            # Si target_x > centre_robot, l'erreur est positive (tourner à droite)
            # Si target_x < centre_robot, l'erreur est négative (tourner à gauche)
            erreur = target_x - centre_robot

            # Publication de l'erreur 
            msg_erreur = Float64()
            msg_erreur.data = float(erreur)
            self.error_publisher.publish(msg_erreur)

            # cv2.imshow("Mon Masque Vert", mask_green)
            cv2.imshow("Mon Masque Rouge", mask_red)
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