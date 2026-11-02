import json
import os
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

# CONSTANTES
FORMAS = ('circulo', 'triangulo', 'hexagono')
TOTAL_FIGURAS = 3

# ATENCAO: o enunciado diz "sequencia crescente de valores em hexadecimal",
# mas o exemplo (vermelho -> verde -> azul = FF0000 -> 00FF00 -> 0000FF) e
# DECRESCENTE em valor numerico. Confirmar com os monitores e ajustar aqui.
ORDEM_CRESCENTE = True

# CONFIRMAR qual som corresponde a qual forma (arquivos em sounds/).
SONS = {
    'circulo': 'circulo.mp3',
    'triangulo': 'triangulo.mp3',
    'hexagono': 'hexagono.mp3',
}


class NodeIntegracao(Node):
    def __init__(self):
        super().__init__('integracao')

        self.figuras = {}  # forma -> valor RGB (int)
        self.finalizado = False

        # Contrato com o nó de visão: String com JSON, ex.:
        # {"forma": "triangulo", "cor": "#FF0000"}
        self.create_subscription(String, '/detected_shapes', self.callback_deteccao, 10)

        # transient_local: quem se inscrever depois ainda recebe a sequência
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_sequencia = self.create_publisher(String, '/sequencia_sons', qos)

        self.get_logger().info('Nó de integração iniciado')

    def callback_deteccao(self, msg):
        if self.finalizado:
            return

        try:
            dado = json.loads(msg.data)
            forma = dado['forma']
            cor = int(dado['cor'].lstrip('#'), 16)
        except (ValueError, KeyError, AttributeError):
            self.get_logger().warn(f'Mensagem de detecção inválida: {msg.data}')
            return

        if not self.adicionar_figura(forma, cor):
            return

        if self.completo():
            self.finalizado = True
            self.executar_sequencia()

    def adicionar_figura(self, forma, cor):
        if forma not in FORMAS:
            self.get_logger().warn(f'Forma não identificada: {forma}')
            return False

        if forma in self.figuras:
            return False  # já registrada, não polui o log a cada frame

        self.figuras[forma] = cor
        self.get_logger().info(f'Figura registrada: {forma} #{cor:06X}')
        return True

    def completo(self):
        return len(self.figuras) == TOTAL_FIGURAS

    def executar_sequencia(self):
        ordem = sorted(
            self.figuras,
            key=self.figuras.get,
            reverse=not ORDEM_CRESCENTE,
        )

        for forma in ordem:
            self.tocar_som(forma)

        msg = String()
        msg.data = ','.join(ordem)
        self.pub_sequencia.publish(msg)
        self.get_logger().info(f'Sequência publicada: {msg.data}')

    def tocar_som(self, forma):
        pasta = os.path.join(get_package_share_directory('pkg_avant'), 'sounds')
        caminho = os.path.join(pasta, SONS[forma])
        self.get_logger().info(f'Tocando {SONS[forma]} ({forma})')
        try:
            subprocess.run(['mpg123', '-q', caminho], check=True)
        except (FileNotFoundError, subprocess.CalledProcessError) as erro:
            self.get_logger().error(f'Falha ao tocar {caminho}: {erro}')


def main(args=None):
    rclpy.init(args=args)
    node = NodeIntegracao()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()