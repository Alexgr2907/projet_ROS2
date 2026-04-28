import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import math

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        
        # 1. RETRAIT DU qos_profile_sensor_data -> On utilise 10
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.repulsion_pub = self.create_publisher(Float64, '/lidar/repulsion', 10)
        self.get_logger().info("🎯 Nœud LIDAR (Esquive Douce) Démarré !")

    def scan_callback(self, msg):
        force_totale = 0.0
        
        # 2. DISTANCE RÉDUITE : On attend d'être à 30 cm pour réagir
        distance_seuil = 0.20  
        
        for i, distance in enumerate(msg.ranges):
            if distance < 0.05 or math.isinf(distance) or math.isnan(distance):
                continue 
            
            angle = i
            if angle > 180:
                angle -= 360
                
            # 3. VISION RÉTRÉCIE : On ne regarde que de -30° à +30°
            # Ça évite de réagir à un obstacle qui n'est pas directement sur notre chemin
            if -45 <= angle <= 45:
                if distance < distance_seuil:
                    # Plus le multiplicateur est petit, plus l'esquive est douce.
                    # On le fixe à 2000 pour une poussée progressive
                    intensite = (distance_seuil - distance) * 2000 
                    
                    if 0 <= angle <= 45: 
                        force_totale += intensite
                    elif -45 <= angle < 0:
                        force_totale -= intensite
                        
        msg_repulsion = Float64()
        msg_repulsion.data = float(force_totale)
        self.repulsion_pub.publish(msg_repulsion)

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()