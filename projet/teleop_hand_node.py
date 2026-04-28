import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import math

# IMPORTS EXPLICITES (Contournement du bug AttributeError MediaPipe)
from mediapipe.python.solutions import hands as mp_hands_module
from mediapipe.python.solutions import drawing_utils as mp_draw_module

class HandTeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_hand_node')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- INITIALISATION DE LA WEBCAM ---
        self.cap = cv2.VideoCapture("http://192.168.0.51:8080/video")        
        
        # --- INITIALISATION DE MEDIAPIPE (Version Robuste UNIQUE) ---
        self.mp_hands = mp_hands_module
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)
        self.mp_draw = mp_draw_module
        
        # Boucle de lecture de la caméra 30 fps pour éviter le retard
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.get_logger().info("En bombe bb ! Ouvrez la fenêtre de la caméra.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Impossible d'ouvrir la caméra (Fermez vos navigateurs web !)")
            return

        # Effet miroir pour un contrôle plus intuitif
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        
        # Conversion BGR (OpenCV) vers RGB (MediaPipe)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        cmd = Twist()
        
        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            
            # 1. DESSIN DU SQUELETTE SUR L'IMAGE
            self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
            
            # Récupération des points clés
            poignet = hand_landmarks.landmark[0]
            bout_pouce = hand_landmarks.landmark[4]
            bout_index = hand_landmarks.landmark[8]
            base_majeur = hand_landmarks.landmark[9]
            
            # --- 2. ACCÉLÉRATEUR : Écartement Pouce - Index ---
            dx = (bout_index.x - bout_pouce.x) * w
            dy = (bout_index.y - bout_pouce.y) * h
            distance_pince = math.hypot(dx, dy)
            
            # --- 3. VOLANT : Inclinaison de la main ---
            # (Un résultat négatif veut dire penché à gauche, positif penché à droite)
            inclinaison_x = base_majeur.x - poignet.x
            
            # Paramètres constants
            seuil_gauche = -0.05
            seuil_droite = 0.05
            vitesse_rotation = 0.4
            vitesse_lineaire = 0.08  # VITESSE CONSTANTE ICI
            
            # --- LOGIQUE CROISÉE (Pince + Inclinaison) ---
            if distance_pince <= 40.0:
                # LA PINCE EST FERMÉE : Le robot n'avance plus.
                cmd.linear.x = 0.0
                
                # Mais il peut toujours pivoter sur lui-même si on incline la main !
                if inclinaison_x < seuil_gauche:
                    cmd.angular.z = vitesse_rotation  
                    cv2.putText(frame, "PIVOT GAUCHE", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 100, 0), 2)
                elif inclinaison_x > seuil_droite:
                    cmd.angular.z = -vitesse_rotation 
                    cv2.putText(frame, "PIVOT DROITE", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 100, 0), 2)
                else:
                    cv2.putText(frame, "STOP TOTAL", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    
            else:
                # LA PINCE EST OUVERTE : Le robot avance à vitesse CONSTANTE
                cmd.linear.x = vitesse_lineaire
                
                if inclinaison_x < seuil_gauche:
                    cmd.angular.z = vitesse_rotation  
                    cv2.putText(frame, "AVANCE + GAUCHE", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                elif inclinaison_x > seuil_droite:
                    cmd.angular.z = -vitesse_rotation 
                    cv2.putText(frame, "AVANCE + DROITE", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                else:
                    cmd.angular.z = 0.0
                    cv2.putText(frame, "AVANCE TOUT DROIT", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
        else:
            # SÉCURITÉ : Si on ne voit pas de main, le robot s'arrête net.
            cv2.putText(frame, "Aucune main detectee", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)

        # Envoi de l'ordre au robot
        self.publisher_.publish(cmd)
        
        # Affichage
        cv2.imshow("Controle Jedi", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HandTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Arrêt sécurisé à la fermeture
        node.publisher_.publish(Twist())
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()