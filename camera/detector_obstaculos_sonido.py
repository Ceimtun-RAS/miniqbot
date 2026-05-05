import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os  # Necesario para ejecutar comandos de sistema

class AlertaObstaculos(Node):
    def __init__(self):
        super().__init__('alerta_obstaculos_cuadrupedo')
        
        # Suscripción al tópico de profundidad alineada
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10)
            
        self.bridge = CvBridge()
        
        # Ruta de tu audio
        self.audio_path = "/home/ieee/Enigma/Enigma_spot/src/media/wowwow.wav"
        
        # Variable de control para no repetir el audio infinitamente (Hysteresis)
        self.audio_reproduciendo = False
        
        self.get_logger().info("🛡️ Escudo virtual activado. Vigilando el frente...")

    def depth_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            h, w = cv_image.shape
            
            # Región de Interés central
            centro_x, centro_y = w // 2, h // 2
            tamano_caja = 50 
            roi = cv_image[centro_y - tamano_caja : centro_y + tamano_caja, 
                           centro_x - tamano_caja : centro_x + tamano_caja]
            
            profundidades_validas = roi[roi > 0]
            
            if profundidades_validas.size > 0:
                distancia_minima_mm = np.min(profundidades_validas)
                
                # --- LÓGICA DE AUDIO (100mm = 10cm) ---
                if distancia_minima_mm < 200:
                    if not self.audio_reproduciendo:
                        self.get_logger().error(f"🚨 CRÍTICO: Objeto a {distancia_minima_mm / 20.0:.1f} cm")
                        
                        # Ejecutamos paplay. El '&' al final permite que el audio suene 
                        # en segundo plano sin congelar tu nodo de ROS2.
                        os.system(f'paplay {self.audio_path} &')
                        
                        self.audio_reproduciendo = True
                
                # Reseteamos el estado cuando el objeto se aleja a más de 15cm
                # Esto evita que el sonido se dispare mil veces por segundo
                elif distancia_minima_mm > 250:
                    self.audio_reproduciendo = False

                # Alerta visual preventiva (opcional, a 30cm)
                elif distancia_minima_mm < 300:
                    self.get_logger().warn(f"Cuidado: Obstáculo a {distancia_minima_mm / 20.0:.1f} cm")

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