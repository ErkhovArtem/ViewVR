import socket
import json
from socketserver import StreamRequestHandler, TCPServer
from maxon_interfaces.srv import Motor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


class DumpHandler(StreamRequestHandler):

    def handle(self) -> None:
        """receive json packets from client"""

        print('connection from {}:{}'.format(*self.client_address))
        try:
            while True:
                data = self.rfile.readline()
                if not data:
                    break
                id = json.loads(data.decode().rstrip())['id']
                angle = json.loads(data.decode().rstrip())['angle']
                if id==1:
                    if (angle>-90) and (angle<90): # set roll angle limits
                        MinimalClientAsync.roll_callback(angle)
                elif id==2:
                    if (angle>-20) and (angle<70): # set pitch angle limits
                        MinimalClientAsync.pitch_callback(angle)
                else:
                    print(f'Invalid motor id: {id}')
        finally:
            print('disconnected from {}:{}'.format(*self.client_address))

roll_pub, pitch_pub = None, None

class MinimalClientAsync(Node):

    def __init__(self):
        global roll_pub, pitch_pub

        super().__init__('tcp_server')
        roll_pub = self.create_publisher(Float32, '/epos_motor_cmd_roll', 10)
        pitch_pub = self.create_publisher(Float32, '/epos_motor_cmd_pitch', 10)
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0.0

        # self.cli = self.create_client(Motor, '/epos_control_service')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req = Motor.Request()
    
    @staticmethod
    def roll_callback(data):
        global roll_pub
        msg = Float32()
        msg.data = float(data)
        roll_pub.publish(msg)

    @staticmethod
    def pitch_callback(data):
        global pitch_pub
        msg = Float32()
        msg.data = float(data)
        pitch_pub.publish(msg)

    def run_tcp_server(self):
        server_ip = "192.168.88.49"
        port = 8000
        server_address = (server_ip, port)
        print('starting up on {}:{}'.format(*server_address))
        with TCPServer(server_address, DumpHandler) as server:
            print('waiting for a connection')
            server.serve_forever()


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
    

