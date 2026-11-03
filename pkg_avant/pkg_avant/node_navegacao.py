import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong


# macros gerais 
ALTURA = 4.0          # altura da varredura (m)
TEMPO_ALVO = 290.0    #tempo médio da missão, pode trocar se quiser
TEMPO_POUSO = 12.0    # estimativa do tempo de descida em LAND (s)
FOLGA_CURVAS = 10.0   # reserva para o drone desacelerar nas curvas (s)
V_MIN = 0.25          # velocidade mínima
V_MAX = 2.0           # velocidade máxima


class NavegacaoNode(Node):

    def __init__(self):
        super().__init__('navegacao_node')
        self.set_pos = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.get_pos = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pos_cb, qos_profile_sensor_data)

        # Publica o estado do drone para os outros nós (visão, integração) lerem.
        self.estado_pub = self.create_publisher(String, '/mission_status', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')

        self.current_pos = PoseStamped()
        self.estado = "INICIO"

        # Arena 14 m (x) por 8 m (y), com o centro no ponto de decolagem
        self.waypoints = self.create_waypoints()
        self.way_index = 0

        self.inicio = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.run_mission)

        # botao de emergencia pelo teclado l para land e r para rtl
        self.emergencia = None
        self.modo_pouso = "LAND"
        self.teclado = threading.Thread(target=self.ler_teclado, daemon=True)
        self.teclado.start()

    def ler_teclado(self):
        self.get_logger().info('Emergência: digite "l" + Enter para LAND ou "r" + Enter para RTL.')
        while True:
            try:
                tecla = input().strip().lower()
            except EOFError:
                return
            if tecla == 'l':
                self.emergencia = "LAND"
            elif tecla == 'r':
                self.emergencia = "RTL"

    def create_waypoints(self):
        waypoints = []
        direction = -1
        for n in range(4):
            y = round(-3.0 + n * 2.0, 2)
            waypoints += [(6.0 * direction, y, ALTURA), (-6.0 * direction, y, ALTURA)]
            direction = -direction
        waypoints.append((0.0, 0.0, ALTURA))  # volta ao centro para pousar
        return waypoints


    def pos_cb(self, msg):
        self.current_pos = msg

    def set_mode(self, mode_name):
        req = SetMode.Request()
        req.custom_mode = mode_name
        return self.set_mode_client.call_async(req)

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        return self.arming_client.call_async(req)

    def takeoff(self, altitude=4.0):
        req = CommandTOL.Request()
        req.altitude = altitude
        return self.takeoff_client.call_async(req)

    def set_speed(self, speed):
        req = CommandLong.Request()
        req.command = 178
        req.param1 = 1.0
        req.param2 = float(speed)
        req.param3 = -1.0
        req.param4 = 0.0
        return self.command_client.call_async(req)

    def send_waypoint(self, x, y, z):
        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.set_pos.publish(msg)

    def distance(self, x, y, z):
        dx = x - self.current_pos.pose.position.x
        dy = y - self.current_pos.pose.position.y
        dz = z - self.current_pos.pose.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def calcular_velocidade(self):
        x = self.current_pos.pose.position.x
        y = self.current_pos.pose.position.y
        comprimento = 0.0
        for wx, wy, wz in self.waypoints:
            comprimento += math.sqrt((wx - x)**2 + (wy - y)**2)
            x, y = wx, wy

        # Tempo que sobra para a varredura = alvo - o que já passou - pouso - folga.
        gasto = (self.get_clock().now() - self.inicio).nanoseconds / 1e9
        restante = TEMPO_ALVO - gasto - TEMPO_POUSO - FOLGA_CURVAS
        velocidade = comprimento / max(restante, 1.0)

        if velocidade > V_MAX:
            self.get_logger().warn(f'Precisaria de {velocidade:.2f} m/s, acima do limite. Usando {V_MAX} m/s e vai passar do tempo.')
            velocidade = V_MAX
        elif velocidade < V_MIN:
            velocidade = V_MIN

        self.get_logger().info(f'Caminho de {comprimento:.0f} m, {gasto:.0f} s já gastos: velocidade {velocidade:.2f} m/s.')
        return velocidade

    def run_mission(self):
        # Se alguém digitou "l" ou "r", vai direto para o pouso, não importa o estado
        if self.emergencia and self.estado != "POUSO":
            self.get_logger().warn(f"EMERGÊNCIA! Modo {self.emergencia} acionado.")
            self.modo_pouso = self.emergencia
            self.estado = "POUSO"

        # a cada ciclo manda o estado no topico (para visao)
        self.estado_pub.publish(String(data=self.estado))

        if self.estado == "INICIO":
            if self.set_mode_client.service_is_ready() and self.arming_client.service_is_ready():
                self.inicio = self.get_clock().now()
                self.future = self.set_mode("GUIDED")
                self.estado = "AGUARDA_GUIDED"
        elif self.estado == "AGUARDA_GUIDED":
            if self.future.done():
                self.estado = "ARM"
        elif self.estado == "ARM":
            self.future = self.arm()
            self.estado = "AGUARDA_ARM"
        elif self.estado == "AGUARDA_ARM":
            if self.future.done():
                if self.future.result().success:
                    self.estado = "TAKEOFF"
                else:
                    self.estado = "ARM"  # não armou, tenta de novo
        elif self.estado == "TAKEOFF":
            if self.takeoff_client.service_is_ready():
                self.future = self.takeoff(ALTURA)
                self.estado = "AGUARDA_TAKEOFF"
        elif self.estado == "AGUARDA_TAKEOFF":
            if self.future.done() and abs(ALTURA - self.current_pos.pose.position.z) <= 0.2:
                self.estado = "SPEED"
        elif self.estado == "SPEED":
            self.future = self.set_speed(self.calcular_velocidade())
            self.estado = "AGUARDA_SPEED"
        elif self.estado == "AGUARDA_SPEED":
            if self.future.done():
                self.estado = "NAVEGACAO"
        elif self.estado == "NAVEGACAO":
            if self.way_index < len(self.waypoints):
                target_x, target_y, target_z = self.waypoints[self.way_index]
                self.send_waypoint(target_x, target_y, target_z)

                dist = self.distance(target_x, target_y, target_z)
                if dist < 0.3:
                    self.get_logger().info(f"Ponto {self.way_index} alcançado, indo para o próximo.")
                    self.way_index += 1
            else:
                self.modo_pouso = "LAND"
                self.estado = "POUSO"

        elif self.estado == "POUSO":
            self.set_mode(self.modo_pouso)
            tempo = (self.get_clock().now() - self.inicio).nanoseconds / 1e9
            self.get_logger().info(f'Modo {self.modo_pouso} enviado após {tempo:.0f} s')
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = NavegacaoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()