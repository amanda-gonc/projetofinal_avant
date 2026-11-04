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

        # ---- CONFIGURAÇÕES DE VISÃO PARA TESTES ----
        self.VISION_DEBUGGING = False  # Usado para simular diferentes condições de visão
        # Os testes foram feitos com alpha=0.7, beta=-20 e com alpha=1.6, beta=30
        self.VISION_ALPHA = 1  # alpha > 1 aumenta contraste, alpha < 1 diminui contraste
        self.VISION_BETA = 0 # beta > 0 clareia, beta < 0 escurece

        # Diversos tipos de ruídos que podem ser testados na imagem. Precisa ter VISION_DEBUGGING = True
        self.GAUSSIAN_NOISE = False
        self.SIGMA = 15.0

        self.SALT_AND_PEPPER = False
        self.AMOUNT = 0.02

        self.SPECKLE = False
        self.STRENGTH = 0.2



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
        
        self.mission_status = None

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coeffs = None

        self.found_shapes = {}  # Dicionário para armazenar as formas encontradas
        self.colors_sent = False  # Flag para indicar se as cores já foram enviadas

        self.call_classic = True 
        self.modelo = YOLO('/home/avant/simulation/starts/avant_ws/src/pkg_avant/models/formas-v2.pt')
        self.get_logger().info('Inscrição ativa, recebendo imagem da camera')

    
    def _add_gaussian_noise(self, frame: cv.typing.MatLike) -> cv.typing.MatLike:
        """Adiciona ruído gaussiano à imagem."""

        rng = np.random.default_rng()
        noise = rng.normal(0, self.SIGMA, frame.shape).astype(np.float32)
        noisy = np.clip(frame.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        return noisy


    def _add_salt_pepper(self, frame: cv.typing.MatLike) -> cv.typing.MatLike:
        """Adiciona ruído sal e pimenta à imagem."""

        rng = np.random.default_rng()
        noisy = frame.copy()
        num = int(self.AMOUNT * frame.size)
        # pixels pretos
        coords = (rng.integers(0, frame.shape[0], num),
                rng.integers(0, frame.shape[1], num),
                rng.integers(0, frame.shape[2], num))
        noisy[coords] = 0
        # pixels brancos
        coords = (rng.integers(0, frame.shape[0], num),
                rng.integers(0, frame.shape[1], num),
                rng.integers(0, frame.shape[2], num))
        noisy[coords] = 255
        return noisy


    def _add_speckle(self, frame: cv.typing.MatLike) -> cv.typing.MatLike:
        """Adiciona ruído speckle à imagem."""

        rng = np.random.default_rng()
        noise = rng.normal(0, self.STRENGTH, frame.shape).astype(np.float32)
        noisy = np.clip(frame.astype(np.float32) * (1 + noise), 0, 255).astype(np.uint8)
        return noisy


    def publish_detected_shapes(self):
        """Publica as formas detectadas no tópico /detected_shapes se a missão estiver finalizada e as cores ainda não foram enviadas."""

        if self.mission_status == 'POUSO' and self.colors_sent == False:
            self.colors_sent = True
            # Guarda os nomes das cores e seus valores hexadecimais em uma lista e ordena pelo valor hexadecimal
            sorted_shapes = [(shape, values) for shape, values in self.found_shapes.items()]
            sorted_shapes.sort(key=lambda x: x[1][1])
            # Publica as formas detectadas no tópico /detected_shapes
            self.get_logger().info("Missão finalizada, publicando formas detectadas...")
            detected_shapes_msg = String()
            detected_shapes_msg.data = f'{sorted_shapes[0][0]} {sorted_shapes[1][0]} {sorted_shapes[2][0]}'
            self.publisher_shapes.publish(detected_shapes_msg)
            self.get_logger().info(f"Formas detectadas publicadas: {detected_shapes_msg.data}")


    def _camera_info_callback(self, msg: CameraInfo):
        """Callback para receber informações da câmera e armazenar a matriz de calibração e os coeficientes de distorção."""

        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)


    def read_im(self, img_raw: Image):
        """Callback para receber imagens da câmera, aplicar correção de distorção e realizar detecção de formas geométricas."""

        img_raw = self.bridge.imgmsg_to_cv2(img_raw, desired_encoding='bgr8')

        # Se a matriz da câmera e os coeficientes de distorção estiverem disponíveis, aplique a correção de distorção
        if self.camera_matrix is not None and self.distortion_coeffs is not None:
            img_cam = cv.undistort(img_raw, self.camera_matrix, self.distortion_coeffs)
        else:
            img_cam = img_raw

        if self.VISION_DEBUGGING:
            if self.VISION_ALPHA != 1 or self.VISION_BETA != 0:
                img_cam = cv.convertScaleAbs(img_cam, alpha=self.VISION_ALPHA, beta=self.VISION_BETA)

            if self.GAUSSIAN_NOISE:
                img_cam = self._add_gaussian_noise(img_cam)

            if self.SALT_AND_PEPPER:
                img_cam = self._add_salt_pepper(img_cam)

            if self.SPECKLE:
                img_cam = self._add_speckle(img_cam)

        # Realiza a detecção de formas geométricas usando a rede neural ou visão clássica, dependendo da configuração
        if self.USE_NEURAL_NETWORK:
            self.geometrics(img_cam)
        if not self.USE_NEURAL_NETWORK or self.call_classic:
            self.classic_vision(img_cam)

        cv.imshow('img_cam', img_cam)
        cv.waitKey(1)


    def geometrics(self, img: cv.typing.MatLike):
        """Realiza a detecção de formas geométricas usando a rede neural YOLOv8."""

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
                    self.get_logger().info(F"Circulo azul detectado pela Rede Neural! HEX = {format(avg_color, '06x')} ({avg_color})")
                elif classe == 1 and 'triângulo' not in self.found_shapes:
                    self.found_shapes['triângulo'] = ['vermelho', avg_color]
                    self.get_logger().info(F"Triângulo vermelho detectado pela Rede Neural! HEX = {format(avg_color, '06x')} ({avg_color})")
                elif classe == 2 and 'hexágono' not in self.found_shapes:
                    self.found_shapes['hexágono'] = ['verde', avg_color]
                    self.get_logger().info(F"Hexagono verde detectado pela Rede Neural! HEX = {format(avg_color, '06x')} ({avg_color})")


    def classic_vision(self, img: cv.typing.MatLike):
        """Realiza a detecção de formas geométricas usando visão clássica baseada em cores e contornos."""

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
                        self.get_logger().info(f"{shape.capitalize()} {color_name} detectado por visão clássica! HEX = {format(avg_color, '06x')} ({avg_color})")


    def mission_status_callback(self,  msg: String):
        """Callback para receber o status da missão do drone."""

        self.mission_status = msg.data
        

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
