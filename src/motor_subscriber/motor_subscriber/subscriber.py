import rclpy
import serial
import signal
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

class MotorSubscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber')
        self.motorSpeeds = [0, 0, 0, 0]
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.subscription = self.create_subscription(
            Int16MultiArray,
            '/motor',
            self.listener_callback,
            10
        )

        # Catch CTRL+C signal
        signal.signal(signal.SIGINT, self.shutdown_motors)

    def listener_callback(self, msg):
        self.motorSpeeds[0] = 246 + int(2 * (msg.data[0] + msg.data[1]))
        self.motorSpeeds[1] = 246 + int(2 * (-msg.data[0] + msg.data[1]))
        self.motorSpeeds[2] = 246 + int(2 * (-msg.data[0] - msg.data[1]))
        self.motorSpeeds[3] = 246 + int(2 * (msg.data[0] - msg.data[1]))
        datas = ','.join(str(data) for data in self.motorSpeeds) + '\n'
        self.ser.write(datas.encode())

    def shutdown_motors(self, signum, frame):
        # Set motor speeds to 0 and send the signal
        self.motorSpeeds = [0, 0, 0, 0]
        datas = ','.join(str(data) for data in self.motorSpeeds) + '\n'
        self.ser.write(datas.encode())
        print("Motors stopped")
        self.ser.close()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    motor_subscriber = MotorSubscriber()
    rclpy.spin(motor_subscriber)
    motor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
