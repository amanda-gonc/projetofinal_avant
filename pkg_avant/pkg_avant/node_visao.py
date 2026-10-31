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
        # ---- DEFINIÇÃO DE CORES EM HSV ----
        # ---- VERMELHO (usa duas faixas por causa do wrap-around do Hue) ----
        self.RED_LOWER1 = [0, 100, 100]
        self.RED_UPPER1 = [10, 255, 255]
        self.RED_LOWER2 = [160, 100, 100]
        self.RED_UPPER2 = [179, 255, 255]

        # ---- VERDE ----
        self.GREEN_LOWER = [45, 150, 100]
        self.GREEN_UPPER = [80, 255, 255]

        # ---- AZUL ----
        self.BLUE_LOWER = [100, 150, 0]
        self.BLUE_UPPER = [140, 255, 255]

        self.COLOR_RANGES = {
            'vermelho': [(self.RED_LOWER1, self.RED_UPPER1), (self.RED_LOWER2, self.RED_UPPER2)],
            'verde': [(self.GREEN_LOWER, self.GREEN_UPPER)],
            'azul': [(self.BLUE_LOWER, self.BLUE_UPPER)]
        }

        self.found_shapes = {}  # Dicionário para armazenar as formas encontradas

        super().__init__('trainee_vision_node')

        self.USE_NEURAL_NETWORK = False  # Defina como True para usar a rede neural, False para visão clássica

        self.subscriber_cam = self.create_subscription(msg_type=Image,
                                                       topic='/camera/image_raw',
                                                       callback=self.read_im,
                                                       qos_profile=10)

        self.bridge = CvBridge()

        self.modelo = YOLO('/home/avant/simulation/starts/avant_ws/src/pkg_avant/models/formas-v2.pt')
        self.get_logger().info('Inscrição ativa, recebendo imagem da camera')


    def read_im(self, img_cv : Image):
        img_cv = self.bridge.imgmsg_to_cv2(img_cv, desired_encoding='bgr8')
        if self.USE_NEURAL_NETWORK:
            self.geometrics(img_cv)
        else:
            self.classic_vision(img_cv)
        cv.imshow('img_cv', img_cv)
        cv.waitKey(1)

    def geometrics(self, img):
        resultado = self.modelo(img, device='cpu', conf=0.5, verbose=False) 
        figuras = resultado[0].boxes 

        for figura in figuras: 
            classe = int(figura.cls)
            # Obtendo as coordenadas do retângulo delimitador da figura e convertendo para inteiros:
            x1, y1, x2, y2 = figura.xyxy[0].tolist()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            if classe == 0 or classe == 1 or classe == 2:
                img_crop_bgr = img[y1:y2, x1:x2]
                img_crop_rgb = cv.cvtColor(img_crop_bgr, cv.COLOR_BGR2RGB)
                img_crop_hsv = cv.cvtColor(img_crop_bgr, cv.COLOR_BGR2HSV)

                # Encontra a cor predominante na imagem recortada em rgb usando a mediana dos pixels
                mask = np.zeros(img_crop_hsv.shape[:2], dtype=np.uint8)
                if classe == 0:
                    for lower, upper in self.COLOR_RANGES['azul']:
                        lower_np = np.array(lower, dtype=np.uint8)
                        upper_np = np.array(upper, dtype=np.uint8)
                        mask += cv.inRange(img_crop_hsv, lower_np, upper_np)
                elif classe == 1:
                    for lower, upper in self.COLOR_RANGES['vermelho']:
                        lower_np = np.array(lower, dtype=np.uint8)
                        upper_np = np.array(upper, dtype=np.uint8)
                        mask += cv.inRange(img_crop_hsv, lower_np, upper_np)
                elif classe == 2:
                    for lower, upper in self.COLOR_RANGES['verde']:
                        lower_np = np.array(lower, dtype=np.uint8)
                        upper_np = np.array(upper, dtype=np.uint8)
                        mask += cv.inRange(img_crop_hsv, lower_np, upper_np)
                pixels_rgb = img_crop_rgb[mask > 0]
                avg_color = np.median(pixels_rgb, axis=0) if len(pixels_rgb) > 0 else np.array([0, 0, 0])

                cv.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv.imshow('img_crop_rgb', img_crop_bgr)
                cv.waitKey(1)

                if classe == 0 and 'círculo' not in self.found_shapes:
                    self.found_shapes['círculo'] = ['azul', avg_color]
                    self.get_logger().info(F"Circulo azul Avistado! HEX = {avg_color}")
                elif classe == 1 and 'triângulo' not in self.found_shapes:
                    self.found_shapes['triângulo'] = ['vermelho', avg_color]
                    self.get_logger().info(F"Triângulo vermelho Avistado! HEX = {avg_color}")
                elif classe == 2 and 'hexágono' not in self.found_shapes:
                    self.found_shapes['hexágono'] = ['verde', avg_color]
                    self.get_logger().info(F"Hexagono verde Avistado! HEX = {avg_color}")


    def classic_vision(self, img):
        hsv_image = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        for color_name, ranges in self.COLOR_RANGES.items():
            mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                lower_np = np.array(lower, dtype=np.uint8)
                upper_np = np.array(upper, dtype=np.uint8)
                mask += cv.inRange(hsv_image, lower_np, upper_np)


            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv.contourArea(contour)
                if area > 500:  # Ajuste o valor conforme necessário
                    x, y, w, h = cv.boundingRect(contour)

                    img_crop_bgr = img[y:y+h, x:x+w]
                    cv.imshow('img_crop', img_crop_bgr)
                    cv.waitKey(1)

                    # Encontra a cor predominante na imagem recortada em rgb usando a mediana dos pixels
                    sub_mask = mask[y:y+h, x:x+w]
                    img_crop_rgb = cv.cvtColor(img_crop_bgr, cv.COLOR_BGR2RGB)
                    pixels_rgb = img_crop_rgb[sub_mask > 0]
                    avg_color = np.median(pixels_rgb, axis=0) if len(pixels_rgb) > 0 else np.array([0, 0, 0])

                    # Desenha o retângulo ao redor da forma detectada
                    cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # Encontra a forma geométrica aproximada (círculo, triângulo ou hexágono)
                    approx = cv.approxPolyDP(contour, 0.04 * cv.arcLength(contour, True), True)
                    shape = "unidentified"
                    if len(approx) == 3 and color_name == "vermelho":
                        shape = "triângulo"
                    elif len(approx) == 6 and color_name == "verde":
                        shape = "hexágono"
                    elif len(approx) > 6 and color_name == "azul":
                        shape = "círculo"
                    
                    # Verifica se a forma já foi detectada antes
                    if shape not in self.found_shapes and shape != "unidentified":
                        self.found_shapes[shape] = [color_name, avg_color]
                        self.get_logger().info(f"{shape.capitalize()} {color_name} detectado! HEX = {avg_color}")



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

