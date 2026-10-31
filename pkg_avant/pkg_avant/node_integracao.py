import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

# CONSTANTES
FORMAS = {0: 'circulo', 1: 'triangulo', 2: 'hexagono'}
TOTAL_FIGURAS = 3


class NodeIntegracao(Node):
    def __init__(self):
        super().__init__("integracao")

        self.figuras = []
        self.finalizado = False

        self.create_subscription(Int32, '/deteccao', self.callback_deteccao, 10) #visao deve publicar a figura que enxerga, coloquei int mas vai depender do que a visao publicar
        self.get_logger().info('Nó de integração iniciado')

    def callback_deteccao(self, msg):
        if self.finalizado:
            return
        forma = msg.data
        self.adicionar_figura(forma)
        if self.completo():
            self.finalizado = True
            self.executar_sequencia()

    def adicionar_figura (self, forma):
        if forma not in FORMAS:
            self.get_logger().warn("Forma não identificada")
            return False
        
        if forma in self.figuras:
            self.get_logger().warn("Forma já existente")
            return False

        self.figuras.append(forma)
        self.get_logger().info("Forma adicionada com sucesso")
        return True

    def completo(self):
        if len(self.figuras) == TOTAL_FIGURAS:
            return True
        
        return False

    def executar_sequencia(self):
        # parte B
        pass



def main(args=None):
    rclpy.init(args=args)
    node = NodeIntegracao()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()