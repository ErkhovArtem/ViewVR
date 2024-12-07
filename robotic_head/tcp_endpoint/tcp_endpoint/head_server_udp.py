import socket
import json
from socketserver import StreamRequestHandler, TCPServer
from maxon_interfaces.srv import Motor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


class MinimalClientAsync(Node):

    def __init__(self):
        global roll_pub, pitch_pub

        super().__init__('tcp_server')
        self.roll_pub = self.create_publisher(Float32, '/epos_motor_cmd_roll', 1)
        self.pitch_pub = self.create_publisher(Float32, '/epos_motor_cmd_pitch', 1)
        # Создаём сокет
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Привязываем сокет к IP-адресу и порту
        self.server_socket.bind(("192.168.1.100", 8000))

    def run_tcp_server(self):
    # Получаем данные от клиента
        try:
            while True:
                data, _ = self.server_socket.recvfrom(1024)
                id = json.loads(data.decode().rstrip())['id']
                angle = json.loads(data.decode().rstrip())['angle']
                if id==1:
                    if (angle>-90) and (angle<90): # set roll angle limits
                        self.roll_callback(angle)
                elif id==2:
                    if (angle>-20) and (angle<70): # set pitch angle limits
                        self.pitch_callback(angle)
                else:
                    print(f'Invalid motor id: {id}')
        finally:
            self.server_socket.close()
    
    def roll_callback(self, data):
        msg = Float32()
        msg.data = float(data)
        print('Published roll: ', data)
        self.roll_pub.publish(msg)

    def pitch_callback(self, data):
        msg = Float32()
        msg.data = float(data)
        print('Published pitch: ', data)
        self.pitch_pub.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_client = MinimalClientAsync()
        print('Node created')
        minimal_client.run_tcp_server()
        rclpy.spin(minimal_client)
        
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
  

if __name__ == '__main__':
    main()
    

