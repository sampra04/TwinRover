import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
import numpy as np
import time
from math import sin, cos


class MPU6050Node(Node):

    def __init__(self):
        super().__init__('mpu6050_node')

        self.sensor = mpu6050(0x68)

        self.publisher = self.create_publisher(Imu, '/imu/data', 10)

        self.timer = self.create_timer(0.02, self.update)   # 50 Hz

        self.last_time = time.time()

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.alpha = 0.98

        self.get_logger().info("MPU6050 IMU Node Started")


    def euler_to_quaternion(self, roll, pitch, yaw):

        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        q = [0,0,0,0]

        q[0] = cr * cp * cy + sr * sp * sy
        q[1] = sr * cp * cy - cr * sp * sy
        q[2] = cr * sp * cy + sr * cp * sy
        q[3] = cr * cp * sy - sr * sp * cy

        return q


    def update(self):

        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        gx = np.deg2rad(gyro['x'])
        gy = np.deg2rad(gyro['y'])
        gz = np.deg2rad(gyro['z'])

        self.roll += gx * dt
        self.pitch += gy * dt
        self.yaw += gz * dt

        ax = accel['x']
        ay = accel['y']
        az = accel['z']

        roll_acc = np.arctan2(ay, az)
        pitch_acc = np.arctan2(-ax, np.sqrt(ay*ay + az*az))

        self.roll = self.alpha*self.roll + (1-self.alpha)*roll_acc
        self.pitch = self.alpha*self.pitch + (1-self.alpha)*pitch_acc

        q = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)

        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        msg.orientation.w = q[0]
        msg.orientation.x = q[1]
        msg.orientation.y = q[2]
        msg.orientation.z = q[3]

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax * 9.81
        msg.linear_acceleration.y = ay * 9.81
        msg.linear_acceleration.z = az * 9.81

        self.publisher.publish(msg)


def main():

    rclpy.init()

    node = MPU6050Node()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
