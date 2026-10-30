import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

# =========== CODIGOS DA REDE NEURAL ==============
# a rede devolve um array com as figuras detectadas em cada frame
# cada forma geométrica é associada a um número
# Circulo = 0
# Triângulo = 1
# Hexagono = 2
# Base de pouso = 3

# IMPORTANTE: Para testar a rede neural na simulação é necessário adicionar a pasta models no pkg_avant do container de voces, é la que a rede fica 

class TraineeVisionNode(Node):

    def __init__(self):
        super().__init__('trainee_vision_node')

        self.subscriber_cam = self.create_subscription(msg_type=Image,
                                                       topic='/camera/image_raw',
                                                       callback=self.read_im,
                                                       qos_profile=10)

        self.bridge = CvBridge()

        self.modelo = YOLO('/home/avant/simulation/starts/avant_ws/src/pkg_avant/models/formas-v2.pt')
        self.get_logger().info('Inscrição ativa, recebendo imagem da camera')


    def read_im(self, img_cam : Image):
        img_cam = self.bridge.imgmsg_to_cv2(img_cam, desired_encoding='bgr8')
        self.geometrics(img_cam)
        cv.imshow('img_cam', img_cam)
        cv.waitKey(1)

    def geometrics(self, img):
        resultado = self.modelo(img, device='cpu', conf=0.5, verbose=False) # A variavel resultado recebe a resposta da rede para o frame atual
        figuras = resultado[0].boxes 
        # figuras recebe a lista de figuras detectadas no frame

        for figura in figuras:
            classe = int(figura.cls)
            x1, y1, x2, y2 = figura.xyxy[0].tolist() # coordenadas da bounding box
            
            cx = (x1 + x2)/2   # centro no eixo x
            cy = (y1 + y2)/2   # centro no eixo y

            Xs = int(cx + (abs(x1 - x2) * 0.1))  # cantos da bounding box
            Ys = int(cy + (abs(y1 - y2) * 0.1))

            Xi = int(cx - (abs(x1 - x2) * 0.1))  
            Yi = int(cy - (abs(y1 - y2) * 0.1))


            if classe == 0 or classe == 1 or classe == 2:
                # recorta um retângulo no centro da bounding box e converte a imagem para RGB
                img_crop_rgb = img[Yi:Ys, Xi:Xs] 
                img_crop_rgb = cv.cvtColor(img_crop_rgb, cv.COLOR_BGR2RGB)

                cv.rectangle(img, (Xs, Ys), (Xi, Yi), (0, 255, 0), 2)
                cv.imshow('img_crop_rgb', img_crop_rgb)
                cv.waitKey(1)

            if classe == 0 :
                self.get_logger().info("Circulo Avistado!", once=True)
            elif classe == 1 :
                self.get_logger().info("Triângulo Avistado!", once=True)
            elif classe == 2 :
                self.get_logger().info("Hexágono Avistado!", once=True)



def main(args=None):
    rclpy.init(args=args)
    node = TraineeVisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
