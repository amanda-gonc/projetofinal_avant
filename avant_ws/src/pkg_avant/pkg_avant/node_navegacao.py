import math
import os
import queue
import select
import signal
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong


# macros gerais 
ALTURA = 4.0          # altura da varredura (m)
TEMPO_ALVO = 290.0    #tempo médio da missão, pode trocar se quiser
TEMPO_POUSO = 15.0    # tempo da descida em LAND (s), medido no teste: 15 s
FOLGA_CURVAS = 5.0    # reserva (s) para o drone chegar ao fim da rota depois da isca
V_MIN = 0.25          # velocidade mínima
V_MAX = 2.0           # velocidade máxima

# estados em que o drone já está pousando (a emergência não reinicia o pouso)
ESTADOS_POUSO = ("POUSO", "AGUARDA_POUSO", "POUSANDO", "FIM")


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

        # A "isca" é um ponto que anda pela rota na velocidade calculada
        # O drone persegue a isca, deixando a liberdade de escolher a velocidade
        self.velocidade = V_MIN
        self.isca = None
        self.t_isca = None

        self.inicio = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.run_mission)

        self.future = None
        self.modo_pouso = "LAND"
        self.espera_ciclos = 0      # pausa entre tentativas de pouso (1 ciclo = 0,1 s)

        # Pedidos de emergência ("LAND" ou "RTL") entram nesta fila, vindos do teclado
        # ou do tópico /emergência
        self.fila_emergencia = queue.SimpleQueue()
        self.emergencia_sub = self.create_subscription(String, '/emergencia', self.emergencia_cb, 10)

        # botao de emergencia pelo teclado: l para land e r para rtl
        self.rodando = True
        self.teclado = threading.Thread(target=self.ler_teclado, daemon=True)
        self.teclado.start()

    def ler_teclado(self):
        # Só lê o teclado se o nó estiver num terminal de verdade (ros2 run)
        # com ros2 launch o teclado não chega ao nó, e um input() ficaria preso
        # para sempre. Nesse caso use o tópico /emergencia
        try:
            tem_terminal = sys.stdin is not None and sys.stdin.isatty()
        except (ValueError, OSError):
            tem_terminal = False
        if not tem_terminal:
            self.get_logger().warn('Sem teclado neste modo (launch?). Para pousar use: '
                                   'ros2 topic pub --once /emergencia std_msgs/msg/String "{data: LAND}"')
            return

        fd = sys.stdin.fileno()
        self.get_logger().info('Emergência: digite "l" + Enter para LAND ou "r" + Enter para RTL.')
        while self.rodando:
            # Espera no máximo 0,2 s por uma tecla. Assim a thread nunca fica presa
            # e termina junto com o nó.
            pronto, _, _ = select.select([fd], [], [], 0.2)
            if not pronto:
                continue
            dados = os.read(fd, 1024)
            if not dados:       # terminal fechado
                return
            for palavra in dados.decode(errors='ignore').split():
                tecla = palavra.lower()
                if tecla == 'l':
                    self.fila_emergencia.put("LAND")
                elif tecla == 'r':
                    self.fila_emergencia.put("RTL")

    def emergencia_cb(self, msg):
        pedido = msg.data.strip().upper()
        if pedido in ("LAND", "RTL"):
            self.fila_emergencia.put(pedido)
        else:
            self.get_logger().warn(f'Pedido "{msg.data}" inválido. Use LAND ou RTL.')

    def pouso_imediato(self):
        # Usado no Ctrl+C: manda LAND e espera a resposta por até 3 s.
        if self.estado in ("POUSANDO", "FIM") and self.modo_pouso == "LAND":
            return      # já está pousando
        self.timer.cancel()     # para de mandar waypoints
        if not self.set_mode_client.service_is_ready():
            self.get_logger().error('Serviço set_mode indisponível: não foi possível mandar LAND.')
            return
        futuro = self.set_mode("LAND")
        rclpy.spin_until_future_complete(self, futuro, timeout_sec=3.0)
        resultado = futuro.result() if futuro.done() else None
        if resultado is not None and resultado.mode_sent:
            self.get_logger().info('LAND aceito. Drone descendo.')
        else:
            self.get_logger().error('LAND NÃO confirmado! Pouse manualmente.')

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

    def mover_isca(self, alvo):
        # Anda com a isca em direção ao waypoint, na velocidade calculada.
        agora = self.get_clock().now()
        dt = min((agora - self.t_isca).nanoseconds / 1e9, 0.5)
        self.t_isca = agora

        # Se o drone ficar mais de 2 m para trás, a isca espera por ele
        if self.distance(*self.isca) > 2.0:
            return

        dx = alvo[0] - self.isca[0]
        dy = alvo[1] - self.isca[1]
        dz = alvo[2] - self.isca[2]
        falta = math.sqrt(dx**2 + dy**2 + dz**2)
        passo = self.velocidade * dt
        if falta <= passo:
            self.isca = alvo        # chegou no waypoint
        else:
            k = passo / falta
            self.isca = (self.isca[0] + dx * k, self.isca[1] + dy * k, self.isca[2] + dz * k)

    def run_mission(self):
        # Se alguém pediu "l" ou "r" (teclado ou tópico), vai direto para o pouso, não importa o estado
        while not self.fila_emergencia.empty():
            pedido = self.fila_emergencia.get_nowait()
            if self.estado in ESTADOS_POUSO and (self.modo_pouso == "LAND" or pedido == self.modo_pouso):
                continue    # já está pousando desse jeito (LAND não vira RTL)
            self.get_logger().warn(f"EMERGÊNCIA! Modo {pedido} acionado.")
            self.modo_pouso = pedido
            self.future = None
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
            self.velocidade = self.calcular_velocidade()
            # limite folgado para o autopilot; quem controla a velocidade de verdade é a isca
            self.future = self.set_speed(max(1.0, 2.0 * self.velocidade))
            self.estado = "AGUARDA_SPEED"
        elif self.estado == "AGUARDA_SPEED":
            if self.future.done():
                # a isca começa onde o drone está agora
                p = self.current_pos.pose.position
                self.isca = (p.x, p.y, p.z)
                self.t_isca = self.get_clock().now()
                self.estado = "NAVEGACAO"
        elif self.estado == "NAVEGACAO":
            if self.way_index < len(self.waypoints):
                alvo = self.waypoints[self.way_index]
                self.mover_isca(alvo)
                self.send_waypoint(*self.isca)    # o drone persegue a isca, não o waypoint

                # Passa para o próximo quando a isca chega no waypoint. No último ponto
                # antes de pousar espera também o drone chegar perto dele
                ultimo = self.way_index == len(self.waypoints) - 1
                if self.isca == alvo and (not ultimo or self.distance(*alvo) < 0.3):
                    self.get_logger().info(f"Ponto {self.way_index} alcançado, indo para o próximo.")
                    self.way_index += 1
            else:
                self.modo_pouso = "LAND"
                self.estado = "POUSO"

        elif self.estado == "POUSO":
            # manda o modo de pouso (LAND ou RTL); se for recusado, tenta de novo
            if self.espera_ciclos > 0:
                self.espera_ciclos -= 1
            elif self.set_mode_client.service_is_ready():
                self.future = self.set_mode(self.modo_pouso)
                self.estado = "AGUARDA_POUSO"
        elif self.estado == "AGUARDA_POUSO":
            if self.future.done():
                try:
                    resultado = self.future.result()
                except Exception:
                    resultado = None
                if resultado is not None and resultado.mode_sent:
                    tempo = (self.get_clock().now() - self.inicio).nanoseconds / 1e9
                    self.get_logger().info(f'Modo {self.modo_pouso} aceito após {tempo:.0f} s')
                    self.estado = "POUSANDO"
                else:
                    self.get_logger().warn(f'Modo {self.modo_pouso} recusado, tentando de novo em 1 s.')
                    self.espera_ciclos = 10
                    self.estado = "POUSO"
        elif self.estado == "POUSANDO":
            if self.current_pos.pose.position.z < 0.2:
                self.get_logger().info('Pousou.')
                self.estado = "FIM"
        elif self.estado == "FIM":
            pass    # continua publicando "FIM" para a visão


def sinal_como_ctrl_c(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Sem o tratador de sinais do rclpy, o Ctrl+C vira KeyboardInterrupt
    # e ainda dá tempo de mandar LAND antes de encerrar.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = NavegacaoNode()
    # O launch encerra os nós com SIGTERM em alguns casos: trata igual ao Ctrl+C.
    signal.signal(signal.SIGTERM, sinal_como_ctrl_c)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn('Ctrl+C: pouso de emergência (LAND).')
        # ignora novos Ctrl+C/SIGTERM (o launch manda mais de um) enquanto o LAND é enviado
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.pouso_imediato()
    finally:
        node.rodando = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()