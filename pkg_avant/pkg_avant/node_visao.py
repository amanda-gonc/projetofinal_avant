import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterType
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
from geometry_msgs.msg import PoseStamped
import time
import math
from ultralytics import YOLO

# =========== CODIGOS DA REDE NEURAL ==============
# o resultado da rede vem em forma de vetor, para cada classe há um número associado

# Circulo Azul = 0
# Triângulo vermelho = 1
# Hexagono verde = 2
# Base de pouso = 3


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


    def read_im(self, img_cv : Image):
        img_cv = self.bridge.imgmsg_to_cv2(img_cv, desired_encoding='bgr8')
        self.geometrics(img_cv)
        cv.imshow('img_cv', img_cv)
        cv.waitKey(1)

    def geometrics(self, img):
        resultado = self.modelo(img, device='cpu', conf=0.5, verbose=False) 
        figuras = resultado[0].boxes 

        for figura in figuras: 
            classe = int(figura.cls)
            x1, y1, x2, y2 = figura.xyxy[0].tolist()

            cx = (x1 + x2)/2   # centro da bounding box no eixo x
            cy = (y1 + y2)/2   # centro da bounding box no eixo y

            Xs = int(cx + (abs(x1 - x2) * 0.1))  # canto superior da bounding box
            Ys = int(cy + (abs(y1 - y2) * 0.1))

            Xi = int(cx - (abs(x1 - x2)* 0.1))  # canto inferior da bounding box
            Yi = int(cy - (abs(y1 - y2) * 0.1))

            if classe == 0 or classe == 1 or classe == 2:
                img_crop_rgb = img[Yi:Ys, Xi:Xs]
                img_crop_rgb = cv.cvtColor(img_crop_rgb, cv.COLOR_BGR2RGB)

                cv.rectangle(img, (Xs, Ys), (Xi, Yi), (0, 255, 0), 2)
                cv.imshow('img_crop_rgb', img_crop_rgb)
                cv.waitKey(1)

                if classe == 0 :
                    self.get_logger().info("Circulo Avistado!", once=True)
                elif classe == 1 :
                    self.get_logger().info("Triângulo vermelho Avistado!", once=True)
                elif classe == 2 :
                    self.get_logger().info("Hexagono verde Avistado!", once=True)



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

