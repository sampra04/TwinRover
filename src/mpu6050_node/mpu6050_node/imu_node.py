def update(self):
    accel = self.sensor.get_accel_data()
    gyro = self.sensor.get_gyro_data()
    current_time = time.time()
    dt = current_time - self.last_time
    self.last_time = current_time

    gx = np.deg2rad(gyro['x'])
    gy = np.deg2rad(gyro['y'])
    gz = np.deg2rad(gyro['z'])

    self.roll  += gx * dt
    self.pitch += gy * dt
    self.yaw   += gz * dt

    ax = accel['x']
    ay = accel['y']
    az = accel['z']

    roll_acc  = np.arctan2(ay, az)
    pitch_acc = np.arctan2(-ax, np.sqrt(ay*ay + az*az))

    self.roll  = self.alpha * self.roll  + (1 - self.alpha) * roll_acc
    self.pitch = self.alpha * self.pitch + (1 - self.alpha) * pitch_acc

    if self.yaw_offset is None:
        self.yaw_offset = self.yaw
        self.get_logger().info(f'Yaw zeroed at startup: {np.degrees(self.yaw_offset):.2f} deg')
    corrected_yaw = self.yaw - self.yaw_offset

    q = self.euler_to_quaternion(self.roll, self.pitch, corrected_yaw)

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
    msg.linear_acceleration.x = ax * self.accel_scale
    msg.linear_acceleration.y = ay * self.accel_scale
    msg.linear_acceleration.z = az * self.accel_scale

    # Covariances — required by robot_localization EKF
    # Row-major 3x3 matrix, diagonal entries are variance per axis
    # Yaw covariance (z) is tightest — gyro integration is our best heading source
    msg.orientation_covariance = [
        0.01, 0.0,  0.0,
        0.0,  0.01, 0.0,
        0.0,  0.0,  0.005   # yaw tighter than roll/pitch
    ]
    # MPU6050 gyro datasheet: ~0.05 deg/s noise → ~8.7e-6 rad²/s² variance
    # Using conservative 0.02 to account for vibration on the rover
    msg.angular_velocity_covariance = [
        0.02, 0.0,  0.0,
        0.0,  0.02, 0.0,
        0.0,  0.0,  0.02
    ]
    # Accel is noisy — we're not fusing it in EKF anyway but must be non-zero
    msg.linear_acceleration_covariance = [
        0.04, 0.0,  0.0,
        0.0,  0.04, 0.0,
        0.0,  0.0,  0.04
    ]

    self.publisher.publish(msg)