import rclpy
from rclpy.node import Node

import sensor_msgs.msg
import geometry_msgs.msg

import math
from icm20948 import ICM20948

class ICMPublisher(Node):
    def __init__(self):
        super().__init__("icm_publisher")
        
        # Parameters
        self.declare_parameter("frame_id", "imu_frame")
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.frame_id = frame_id

        self.declare_parameter("pub_rate", 50)
        pub_rate = self.get_parameter("pub_rate").get_parameter_value().integer_value
        self.pub_rate = pub_rate

        self.declare_parameter("pitch_filter_gain", 0.8)
        pitch_filter_gain = self.get_parameter("pitch_filter_gain").get_parameter_value().integer_value
        self.pitchGyroFavoring = pitch_filter_gain

        self.declare_parameter("roll_filter_gain", 0.8)
        roll_filter_gain = self.get_parameter("roll_filter_gain").get_parameter_value().integer_value
        self.rollGyroFavoring = roll_filter_gain
        
        # IMU instance
        self.imu = ICM20948()

        self.imu.set_gyro_full_scale(2000)
        self.imu.set_accelerometer_full_scale(16)

        self.pitch = 0
        self.roll = 0
        self.yaw = 0

        # Store the previous time for integration
        self.last_time = self.get_clock().now()

        # Publishers
        self.imu_pub_ = self.create_publisher(sensor_msgs.msg.Imu, "/imu/data_raw", 10)
        self.mag_pub_ = self.create_publisher(sensor_msgs.msg.MagneticField, "/imu/mag", 10)
        self.angles_pub = self.create_publisher(geometry_msgs.msg.Vector3, "/imu/angles", 10)
        
        self.pub_clk_ = self.create_timer(1 / self.pub_rate, self.publish_cback)

    def angles_from_data(self, ax, ay, az, gx, gy, gz, dt):
        # Acceleration-based angles (from accelerometer data)
        pitchFromAccel = math.atan2(-ax, math.sqrt(ay ** 2 + az ** 2)) * (180 / math.pi)
        rollFromAccel = math.atan2(ay, math.sqrt(ax ** 2 + az ** 2)) * (180 / math.pi)

        # Gyro integration (angles from gyroscope)
        self.pitch += gy * dt  # integrating gyro angular velocity over time
        self.roll += gx * dt
        self.yaw += gz * dt

        # Complementary filter
        pitch = self.pitchGyroFavoring * self.pitch + (1 - self.pitchGyroFavoring) * pitchFromAccel
        roll = self.rollGyroFavoring * self.roll + (1 - self.rollGyroFavoring) * rollFromAccel
        yaw = self.yaw  # No accelerometer data for yaw, so we rely fully on the gyro

        return pitch, roll, yaw

    def publish_cback(self):
        # Time since the last update
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # convert to seconds
        self.last_time = current_time

        imu_msg = sensor_msgs.msg.Imu()
        mag_msg = sensor_msgs.msg.MagneticField()
        angles_msg = geometry_msgs.msg.Vector3()
        
        mx, my, mz = self.imu.read_magnetometer_data()
        ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        imu_msg.linear_acceleration.x = ax * 9.81 / 2048.0
        imu_msg.linear_acceleration.y = ay * 9.81 / 2048.0
        imu_msg.linear_acceleration.z = az * 9.81 / 2048.0
        imu_msg.angular_velocity.x = gx * math.pi / (16.4 * 180)
        imu_msg.angular_velocity.y = gy * math.pi / (16.4 * 180)
        imu_msg.angular_velocity.z = gz * math.pi / (16.4 * 180)
        imu_msg.orientation_covariance[0] = -1

        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = self.frame_id
        mag_msg.magnetic_field.x = mx * 1e-6 / 0.15
        mag_msg.magnetic_field.y = my * 1e-6 / 0.15
        mag_msg.magnetic_field.z = mz * 1e-6 / 0.15

        # Compute angles
        self.pitch, self.roll, self.yaw = self.angles_from_data(ax, ay, az, gx, gy, gz, dt)
        angles_msg.x, angles_msg.y, angles_msg.z = self.pitch, self.roll, self.yaw

        # Publish messages
        self.imu_pub_.publish(imu_msg)
        self.mag_pub_.publish(mag_msg)
        self.angles_pub.publish(angles_msg)

def main(args=None):
    rclpy.init(args=args)
    icm_publisher = ICMPublisher()
    rclpy.spin(icm_publisher)

    icm_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
