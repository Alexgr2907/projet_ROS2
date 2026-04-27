import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import time

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.listener_callback, 10)
        self.error_publisher = self.create_publisher(Float64, '/vision/direction_erreur', 10)
        self.etat_publisher = self.create_publisher(Int32, '/etat_mission', 10) 
        self.choix_rond_point = 'droite'
        # Mémoire de la largeur de la route
        self.moitie_route = 200.0
        # Mémoire des challenges
        self.challenge_actuel = 1
        self.temps_dernier_bleu = 0.0

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

    def detecter_transition_bleue(self, mask_blue):
        """Détection de la ligne bleue par comptage de pixels (Radar de sol)"""
        h, w = mask_blue.shape
        
        # On ne garde que les 30% tout en bas de l'image (devant les roues)
        limite_haute = int(h * 0.6)
        zone_basse = mask_blue[limite_haute:h, :]
        
        # On compte littéralement le nombre de pixels bleus allumés
        pixels_bleus = cv2.countNonZero(zone_basse)
        
        # S'il y a plus de 500 pixels bleus
        if pixels_bleus > 500:
            return True
            
        return False

    def listener_callback(self, msg):
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erreur de conversion d'image : {e}")
            return

        h, w, _ = image.shape
        roi = image[int(h * 0.6):h, 0:w] # Zone d'intérêt (ROI)
        h_roi = roi.shape[0]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask_green = cv2.inRange(hsv, np.array([45, 50, 20]), np.array([95, 255, 255]))
        mask_red1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
        mask_red2 = cv2.inRange(hsv, np.array([170, 70, 50]), np.array([179, 255, 255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_blue = cv2.inRange(hsv, np.array([100, 100, 50]), np.array([140, 255, 255]))

        # 1. Analyse avec le nouveau filtre de hauteur h_roi
        cx_green, mur_vert, box_green = self.analyser_ligne(mask_green, w, h_roi)
        cx_red, mur_rouge, box_red = self.analyser_ligne(mask_red, w, h_roi)

        # 2. Détection de la ligne bleue
        transition_franchie = self.detecter_transition_bleue(mask_blue)
        
        if transition_franchie:
            # Sécurité anti-spam : 5 secondes d'attente entre deux lignes
            if time.time() - self.temps_dernier_bleu > 5.0:
                self.challenge_actuel += 1
                self.temps_dernier_bleu = time.time()
                self.get_logger().warn(f"🟦 LIGNE BLEUE FRANCHIE ! Passage au Challenge {self.challenge_actuel}")

        # On publie l'état en permanence
        msg_etat = Int32()
        msg_etat.data = self.challenge_actuel
        self.etat_publisher.publish(msg_etat)

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
            c_g = (0, 0, 255) if mur_vert else (0, 255, 0)
            cv2.rectangle(roi, (gx, gy), (gx+gw, gy+gh), c_g, 2)
        if box_red:
            rx, ry, rw, rh = box_red
            c_r = (255, 0, 0) if mur_rouge else (0, 0, 255)
            cv2.rectangle(roi, (rx, ry), (rx+rw, ry+rh), c_r, 2)

        cv2.circle(roi, (int(target_x), int(roi.shape[0]/2)), 8, (255, 0, 0), -1)

        # Affichage du texte Challenge
        cv2.putText(roi, f"CHALLENGE : {self.challenge_actuel}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Affichage prolongé de l'alerte bleue (pendant 2 secondes)
        if time.time() - self.temps_dernier_bleu < 2.0:
            cv2.putText(roi, "LIGNE BLEUE FRANCHIE !", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow("Camera Robot", roi)

        masques_combines = np.hstack((mask_green, mask_red, mask_blue))
        cv2.imshow("Masques (Vert | Rouge | Bleu)", masques_combines)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
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