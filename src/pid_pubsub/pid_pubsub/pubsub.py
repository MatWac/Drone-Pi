import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16MultiArray
import time
import math


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()

    def calculate(self, setpoint, measured_value):
        now = time.time()
        time_difference = now - self.last_time
        self.last_time = now

        error = measured_value - setpoint
        self.integral += error * time_difference
        derivative = (error - self.last_error) / time_difference
        self.last_error = error

        # Ajouter des logs pour vérifier l'erreur et la sortie du PID
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        return int(output)

class PIDPubSub(Node):
    def __init__(self):
        super().__init__('pid_pubsub')

        self.declare_parameter("kp", 1.1)
        kp = self.get_parameter("kp").get_parameter_value().double_value
        self.kp = kp

        self.declare_parameter("ki", 0.0)
        ki = self.get_parameter("ki").get_parameter_value().double_value
        self.ki = ki

        self.declare_parameter("kd", 0.0)
        kd  = self.get_parameter("kd").get_parameter_value().double_value
        self.kd = kd

        self.pid = PIDController(self.kp, self.ki, self.kd)
        
        self.subscription = self.create_subscription(
            Vector3,
            'imu/angles',
            self.topic_callback,
            10)
        
        self.publisher = self.create_publisher(Int16MultiArray, 'motor', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.angles = [0, 0]

    def topic_callback(self, msg):
        # Extraire les composants quaternion du message IMU
        pitch = msg.x
        roll = msg.y

        # Stocker les angles roll et pitch
        self.angles[0] = pitch
        self.angles[1] = roll

    def timer_callback(self):
        message = Int16MultiArray()
        message.data = [
            self.pid.calculate(0.0, abs(self.angles[0])),
            self.pid.calculate(0.0, abs(self.angles[1]))
        ]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    pid_pubsub = PIDPubSub()
    rclpy.spin(pid_pubsub)
    pid_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
