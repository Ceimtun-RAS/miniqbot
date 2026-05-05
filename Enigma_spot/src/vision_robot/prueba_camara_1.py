import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CameraDetector(Node):
    def __init__(self):
        super().__init__('camera_detector')
        self.bridge = CvBridge()
        # Suscripción al tópico de profundidad
        self.subscription = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.listener_callback, 10)
        self.triggered = False

    def listener_callback(self, msg):
        # Convertir mensaje ROS a imagen OpenCV
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # Obtener valor del centro de la imagen (distancia en mm)
        height, width = depth_image.shape
        center_dist = depth_image[height//2, width//2]

        # Lógica de detección (150 mm = 15 cm)
        if 0 < center_dist < 150:
            if not self.triggered:
                self.get_logger().info('¡Objeto detectado a < 15cm!')
                os.system('play alerta.mp3') # O tu comando de audio
                self.triggered = True
        else:
            self.triggered = False

def main(args=None):
    rclpy.init(args=args)
    node = CameraDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()