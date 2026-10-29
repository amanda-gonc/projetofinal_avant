import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, String


class NodeIntegracao(Node):
    def __init__(self):
        super(). __init__(node_integracao)

        self.create_subscription(Int32, '/deteccao', self.callback_deteccao, 10) #visao deve publicar a figura que enxerga, coloquei int mas vai depender do que a visao publicar
        self.pub_sequencia = self.create_publisher(String, '/sequencia_sons', 10) #saida de sons
        self.get_logger().info('Nó de integração iniciado')

    def callback_deteccao(self, msg):
        # por enquanto deixei com pass até implementar
        pass

    def executar_sequencia(self):
        sequencia = self.registro.sequencia()
        # tocar o áudio de cada classe, na ordem de `sequencia`
        # publicar em self.pub_sequencia os audios
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