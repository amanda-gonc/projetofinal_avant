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


class NavegacaoNode(Node):

    def __init__(self):
        super().__init__('navegacao_node')
        self.set_pos = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local',10)
        self.get_pos = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pos_cb,qos_profile_sensor_data)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        
        self.current_pos = PoseStamped()
        self.estado = "INICIO"

        self.waypoints = [(-6.1,-3.3,4.0),(-6.1,3.3,4.0),(-4.575,3.3,4.0),(-4.575,-3.3,4.0),(-3.05,-3.3,4.0),(-3.05,3.3,4.0),(-1.525,3.3,4.0),(-1.525,-3.3,4.0),(0.0,-3.3,4.0),(0.0,3.3,4.0),(1.525,3.3,4.0),(1.525,-3.3,4.0),(3.05,-3.3,4.0),(3.05,3.3,4.0),(4.575,3.3,4.0),(4.575,-3.3,4.0),(6.1,-3.3,4.0),(6.1,3.3,4.0)]
        self.way_index = 0

        self.timer = self.create_timer(0.1, self.run_mission)
        self.tempo_armado = None

    
    def pos_cb(self,msg):
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

    def pouso_rtl(self):
        self.get_logger().info("Solicitando modo RTL (Retorno à Base)...")
        self.set_mode("RTL") 

    def send_waypoint(self,x,y,z):
        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.set_pos.publish(msg)

    def distance(self, x, y, z):
        dx = x - self.current_pos.pose.position.x
        dy = y - self.current_pos.pose.position.y
        dz = z - self.current_pos.pose.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def run_mission(self):
        if self.estado == "INICIO":
            if self.set_mode_client.service_is_ready() and self.arming_client.service_is_ready():
                self.future = self.set_mode("GUIDED")
                self.estado = "AGUARDA_GUIDED"
        elif self.estado == "AGUARDA_GUIDED":
            if self.future.done():
                self.estado = "ARM"
                self.tempo_armado = self.get_clock().now()
        elif self.estado == "ARM":    
            self.future = self.arm()
            self.estado = "AGUARDA_ARM" 
        elif self.estado == "AGUARDA_ARM":
            if self.future.done():  
                self.estado = "TAKEOFF"
        elif self.estado == "TAKEOFF":
            if self.takeoff_client.service_is_ready():
                self.future = self.takeoff(4.0)
                self.estado = "AGUARDA_TAKEOFF"
        elif self.estado == "AGUARDA_TAKEOFF":
            if self.future.done() and abs(4.0-self.current_pos.pose.position.z)<=0.2:
                self.estado = "SPEED"
        elif self.estado == "SPEED":
            self.future = self.set_speed(4.0)
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
                self.estado = "LAND"

        elif self.estado == "LAND":
            self.pouso_rtl()
            self.get_logger().info('Missão finalizada.')
            self.timer.cancel()



def main(args=None):
    rclpy.init(args=args)
    node = NavegacaoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Falha detectada, pouso emergencial acionado.") 
        node.pouso_rtl()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()