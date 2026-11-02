import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from ultralytics import YOLO

# =========== CODIGOS DA REDE NEURAL ==============
# a rede devolve um array com as figuras detectadas em cada frame
# cada forma geométrica é associada a um número

# Circulo = 0
# Triângulo = 1
# Hexagono = 2
# Base de pouso = 3

# IMPORTANTE: Para testar a rede neural na simulação é necessário adicionar a pasta models no pkg_avant do container de voces, é la que a rede fica 

# =================================================


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

        self.USE_NEURAL_NETWORK = True  # Defina como True para usar a rede neural, False para visão clássica
        self.MIN_CONF_NETWORK = 0.95 # Valor minimo de confiança que a rede deve ter para ser utilizada  

        self.subscriber_cam = self.create_subscription(msg_type=Image,
                                                       topic='/camera/image_raw',
                                                       callback=self.read_im,
                                                       qos_profile=10)
        
        self.camera_info_subscriber = self.create_subscription(msg_type=CameraInfo,
                                                               topic='/camera/camera_info',
                                                               callback=self._camera_info_callback,
                                                               qos_profile=10)

        # Publisher para publicar as formas detectadas
        self.publisher_shapes = self.create_publisher(msg_type=String,
                                                      topic='/detected_shapes',
                                                      qos_profile=10)
        
        # Timer para publicar as formas detectadas a cada 1 segundo
        self.timer = self.create_timer(1.0, self.publish_detected_shapes)

        # Subscriber para saber o estado da missão (se o drone pousou ou não)
        self.status_subscriber = self.create_subscription(msg_type=String,
                                                          topic='/mission_status',
                                                          callback=self.mission_status_callback,
                                                          qos_profile=10)
        

        self.mission_status = ""
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coeffs = None

        self.call_classic = True 
        self.modelo = YOLO('/home/avant/simulation/starts/avant_ws/src/pkg_avant/models/formas-v2.pt')
        self.get_logger().info('Inscrição ativa, recebendo imagem da camera')


    def publish_detected_shapes(self):
        if self.mission_status == 'LAND':
            # Guarda os nomes das cores e seus valores hexadecimais em uma lista e ordena pelo valor hexadecimal
            sorted_shapes = [(color, hex) for color, hex in self.found_shapes.items()]
            sorted_shapes.sort(key=lambda x: x[1])
            # Publica as formas detectadas no tópico /detected_shapes
            self.get_logger().info("Missão finalizada, publicando formas detectadas...")
            detected_shapes_msg = String()
            detected_shapes_msg.data = f'{sorted_shapes[0][0]} {sorted_shapes[1][0]} {sorted_shapes[2][0]}'
            self.publisher_shapes.publish(detected_shapes_msg)
            self.get_logger().info(f"Formas detectadas publicadas: {self.found_shapes}")

    def _camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)


    def read_im(self, img_raw : Image):
        img_raw = self.bridge.imgmsg_to_cv2(img_raw, desired_encoding='bgr8')

        # Se a matriz da câmera e os coeficientes de distorção estiverem disponíveis, aplique a correção de distorção
        if self.camera_matrix is not None and self.distortion_coeffs is not None:
            img_cam = cv.undistort(img_raw, self.camera_matrix, self.distortion_coeffs)
        else:
            img_cam = img_raw

        # Realiza a detecção de formas geométricas usando a rede neural ou visão clássica, dependendo da configuração
        if self.USE_NEURAL_NETWORK:
            self.geometrics(img_cam)
        if not self.USE_NEURAL_NETWORK or self.call_classic:
            self.classic_vision(img_cam)

        cv.imshow('img_cam', img_cam)
        cv.waitKey(1)


    def geometrics(self, img):
        # A variavel resultado recebe a resposta da rede para o frame atual
        resultado = self.modelo(img, device='cpu', conf=0.5, verbose=False)
        # figuras recebe a lista de figuras detectadas no frame
        figuras = resultado[0].boxes 
        # O atributo decide se o método de visão clássica é necessário para determinado frame 
        self.call_classic = False 

        for figura in figuras: 
            classe = int(figura.cls)
            conf = float(figura.conf)

            if conf < self.MIN_CONF_NETWORK:
                self.call_classic = True
                continue 

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
                avg_color = int(avg_color[0]) << 16 | int(avg_color[1]) << 8 | int(avg_color[2])

                cv.imshow('img_crop', img_crop_bgr)
                cv.waitKey(1)

                if classe == 0 and 'círculo' not in self.found_shapes:
                    self.found_shapes['círculo'] = ['azul', avg_color]
                    self.get_logger().info(F"Circulo azul detectado pela Rede Neural! HEX = {avg_color}")
                elif classe == 1 and 'triângulo' not in self.found_shapes:
                    self.found_shapes['triângulo'] = ['vermelho', avg_color]
                    self.get_logger().info(F"Triângulo vermelho detectado pela Rede Neural! HEX = {avg_color}")
                elif classe == 2 and 'hexágono' not in self.found_shapes:
                    self.found_shapes['hexágono'] = ['verde', avg_color]
                    self.get_logger().info(F"Hexagono verde detectado pela Rede Neural! HEX = {avg_color}")


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
                    avg_color = int(avg_color[0]) << 16 | int(avg_color[1]) << 8 | int(avg_color[2])

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
                        self.get_logger().info(f"{shape.capitalize()} {color_name} detectado por visão clássica! HEX = {avg_color}")


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

