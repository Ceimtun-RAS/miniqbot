import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class AlertaObstaculos(Node):
    def __init__(self):
        super().__init__('alerta_obstaculos_cuadrupedo')
        
        # Nos suscribimos al tópico de profundidad alineada
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10)
            
        self.bridge = CvBridge()
        self.get_logger().info("Escudo virtual activado. Vigilando el frente a < 30cm...")

    def depth_callback(self, msg):
        try:
            # Convertimos el mensaje de ROS2 a una matriz de NumPy (OpenCV)
            # La D435i entrega profundidad en formato de 16 bits sin signo (milímetros)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Obtenemos las dimensiones de la imagen (usualmente 640x480 o 848x480)
            h, w = cv_image.shape
            
            # Definimos una Región de Interés (ROI) en el centro de la cámara
            # Evaluamos un cuadro de 100x100 píxeles justo al frente del cuadrúpedo
            centro_x, centro_y = w // 2, h // 2
            tamano_caja = 50 
            
            roi = cv_image[centro_y - tamano_caja : centro_y + tamano_caja, 
                           centro_x - tamano_caja : centro_x + tamano_caja]
            
            # Filtramos los valores en 0 (píxeles donde el láser infrarrojo falló en leer)
            profundidades_validas = roi[roi > 0]
            
            if profundidades_validas.size > 0:
                # Buscamos el punto más cercano dentro de nuestro cuadro central
                distancia_minima_mm = np.min(profundidades_validas)
                
                # Regla de detección: Menos de 300 milímetros (30 cm)
                if distancia_minima_mm < 300:
                    self.get_logger().warn(f"¡ALERTA DE COLISIÓN! Obstáculo a {distancia_minima_mm / 10.0:.1f} cm")
                    
                    # AQUÍ IRÍA TU LÓGICA DE EVASIÓN
                    # Ejemplo: publicar un mensaje 'Twist' en /cmd_vel con velocidad lineal = 0.0

        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    nodo_alerta = AlertaObstaculos()
    
    try:
        rclpy.spin(nodo_alerta)
    except KeyboardInterrupt:
        pass
    finally:
        nodo_alerta.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()