import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

# =========== CODIGOS DA REDE NEURAL ==============
# o resultado da rede vem em forma de vetor, para cada classe há um número associado

# Circulo Azul = 0
# Triângulo vermelho = 1
# Hexagono verde = 2
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
        # self.count = 0
        # self.timer = self.create_timer(5.0, self.im_write) 
        # # contador e timer do metodo de tirar fotos 

        self.modelo = YOLO('/home/avant/simulation/starts/avant_ws/src/pkg_avant/models/formas-v2.pt')
        self.get_logger().info('Inscrição ativa, recebendo imagem da camera')


    def read_im(self, img_cv : Image):
        img_cv = self.bridge.imgmsg_to_cv2(img_cv, desired_encoding='bgr8')
        # self.im_write(img_cv) # chama o metodo de fotografar
        self.geometrics(img_cv)
        cv.imshow('img_cv', img_cv)
        cv.waitKey(1)

    def geometrics(self, img):
        # Recomendo escrever a identificacao por visao classica aqui para facilitar a combinação das lógicas

        resultado = self.modelo(img, device='cpu', conf=0.5, verbose=False) # variavel resultado recebe o vetor de resposta da rede
        figuras = resultado[0].boxes 

        for figura in figuras: # verifica se a figura foi avistada em algum momento, ou seja, se ela esta no vetor 
            classe = int(figura.cls)

            # Aqui acho que temos que criar um server para mandar uma mensagem para o nó de integração avisando a figura e a cor avistada 

            if classe == 0 :
                self.get_logger().info("Circulo azul Avistado!", once=True)
            elif classe == 1 :
                self.get_logger().info("Triângulo vermelho Avistado!", once=True)
            elif classe == 2 :
                self.get_logger().info("Hexagono verde Avistado!", once=True)

    # codigo para tirar fotos, se precisar
    # def im_write(self, frame):
    #    self.count += 1
    #    cv.imwrite(f'/home/avant/simulation/starts/avant_ws/dataset/frame_{self.count}.png', img_p)

    # Pensei em depois tirarmos mais fotos aplicando os filtros que voce tinha comentado que usou na sua tarefa de visao para fazer o fine-tuning da rede. So não aplique nenhum que altere a cor da imagem pra nao confundir ela.


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
