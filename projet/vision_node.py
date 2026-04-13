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
        self.choix_rond_point = 'gauche'
        # Mémoire de la largeur de la route
        self.moitie_route = 200.0 

    def analyser_ligne(self, mask, w_img, h_roi):
        """Détecte si une ligne forme un vrai mur avec filtres de sécurité"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            aire_contour = cv2.contourArea(c)
            
            if aire_contour > 600: # Un peu plus de pixels pour filtrer le bruit
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00']) # On récupère aussi la hauteur du centre
                    
                    x, y, w, h = cv2.boundingRect(c)
                    aire_box = w * float(h)
                    extent = aire_contour / max(1.0, aire_box)
                    
                    # CONDITIONS DE MUR (Angle Droit) :
                    # 1. La forme est bien horizontale (w > h)
                    est_aplati = w > h * 2.0
                    # 2. Elle remplit bien sa boîte (pas une diagonale)
                    est_plein = extent > 0.50 
                    # 3. Elle est loin devant (dans le haut du ROI)
                    est_loin = cy < (h_roi * 0.4) 
                    
                    # C'est un mur uniquement si c'est aplati, plein ET loin
                    est_un_mur = est_aplati and est_plein and est_loin
                    
                    return cx, est_un_mur, (x, y, w, h)
        return None, False, None

    def listener_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e: return

        h, w, _ = image.shape
        roi = image[int(h * 0.5):h, 0:w] # Zone d'intérêt (ROI)
        h_roi = roi.shape[0]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask_green = cv2.inRange(hsv, np.array([35, 50, 20]), np.array([85, 255, 255]))
        mask_red1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
        mask_red2 = cv2.inRange(hsv, np.array([170, 70, 50]), np.array([179, 255, 255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # 1. Analyse avec le nouveau filtre de hauteur h_roi
        cx_green, mur_vert, box_green = self.analyser_ligne(mask_green, w, h_roi)
        cx_red, mur_rouge, box_red = self.analyser_ligne(mask_red, w, h_roi)

        centre_image = w / 2
        erreur_finale = 0.0
        target_x = centre_image 

        # 2. LOGIQUE DE DÉCISION
        if mur_vert:
            erreur_finale = 400.0 
            target_x = centre_image + 400 
        elif mur_rouge:
            erreur_finale = -400.0
            target_x = centre_image - 400
        else:
            # 3. CONDUITE NORMALE ET ROND-POINT
            if cx_green is not None and cx_red is not None:
                
                # --- LE DÉCLENCHEUR INFAILLIBLE DU ROND-POINT ---
                # Si le Rouge est à gauche du Vert, on regarde l'îlot central !
                if cx_red < cx_green:
                    if self.choix_rond_point == 'gauche':
                        # On s'engage à GAUCHE : on vise à gauche du demi-cercle Rouge
                        target_x = cx_red - self.moitie_route
                        cv2.putText(roi, "ROND-POINT: GO GAUCHE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    else:
                        # On s'engage à DROITE : on vise à droite du demi-cercle Vert
                        target_x = cx_green + self.moitie_route
                        cv2.putText(roi, "ROND-POINT: GO DROITE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # --- CONDUITE CLASSIQUE EN LIGNE DROITE ---
                else:
                    target_x = (cx_green + cx_red) / 2
                    largeur = (cx_red - cx_green) / 2.0
                    # On met à jour la mémoire si la route a une taille cohérente
                    if largeur < 300.0: 
                        self.moitie_route = largeur

            # --- S'IL NE VOIT QU'UNE SEULE LIGNE (Dans le rond-point ou en courbe) ---
            elif cx_green is not None:
                target_x = cx_green + self.moitie_route
            elif cx_red is not None:
                target_x = cx_red - (self.moitie_route * 1.2)
                
            erreur_finale = target_x - centre_image
            
        msg_erreur = Float64()
        msg_erreur.data = float(erreur_finale)
        self.error_publisher.publish(msg_erreur)

        # Dessin et Affichage
        if box_green:
            gx, gy, gw, gh = box_green
            c_g = (0,0,255) if mur_vert else (0,255,0)
            cv2.rectangle(roi, (gx,gy), (gx+gw, gy+gh), c_g, 2)
        if box_red:
            rx, ry, rw, rh = box_red
            c_r = (255,0,0) if mur_rouge else (0,0,255)
            cv2.rectangle(roi, (rx,ry), (rx+rw, ry+rh), c_r, 2)

        cv2.circle(roi, (int(target_x), int(roi.shape[0]/2)), 8, (255, 0, 0), -1)
        cv2.imshow("Camera Robot", roi)
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