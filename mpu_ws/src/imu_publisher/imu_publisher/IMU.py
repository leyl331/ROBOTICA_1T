import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R

import time
import numpy as np


class ImuToOdom(Node):
    def __init__(self):
        super().__init__('imu_to_odom')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.05, self.update)  # 20Hz

        # Inicializar IMU
        self.mpu = MPU9250(
            address_ak=None,
            address_mpu_master=MPU9050_ADDRESS_68,
            address_mpu_slave=None,
            bus=1,
            gfs=GFS_250,
            afs=AFS_2G
        )

        # Configurar manualmente (opcional)
        self.mpu.bus.write_byte_data(self.mpu.address_mpu_master, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self.mpu.bus.write_byte_data(self.mpu.address_mpu_master, GYRO_CONFIG, GFS_250 << 3)
        self.mpu.bus.write_byte_data(self.mpu.address_mpu_master, ACCEL_CONFIG, AFS_2G << 3)
        self.mpu.ares = 2.0 / 32768.0
        self.mpu.gres = 250.0 / 32768.0

        # Estado inicial
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.ahrs = Madgwick(beta=0.02)  #  Más suave (antes era beta=0.1)
        self.prev_time = time.time()
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)

        # Parámetros del filtro EMA y umbral
        self.ema_alpha = 0.05  #  Más conservador (antes era 0.1)
        self.velocity_ema = np.zeros(3)
        self.position_ema = np.zeros(3)
        self.acc_threshold = 0.15  #  Filtrado más agresivo (antes 0.09)

    def update(self):
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        accel = np.array(self.mpu.readAccelerometerMaster())
        gyro = np.deg2rad(self.mpu.readGyroscopeMaster())  # rad/s

        # Actualizar filtro Madgwick
        self.q = self.ahrs.updateIMU(self.q, gyr=gyro, acc=accel)

        # Transformar aceleración al marco global
        r = R.from_quat([self.q[1], self.q[2], self.q[3], self.q[0]])
        acc_global = r.apply(accel)
        acc_global[2] -= 9.81  # quitar gravedad

        # Filtro de umbral
        acc_global[np.abs(acc_global) < self.acc_threshold] = 0.0

        # Integración y filtro EMA
        self.velocity += acc_global * dt
        self.velocity_ema = self.ema_alpha * self.velocity + (1 - self.ema_alpha) * self.velocity_ema

        self.position += self.velocity_ema * dt
        self.position_ema = self.ema_alpha * self.position + (1 - self.ema_alpha) * self.position_ema

        # Mensaje de odometría
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = Clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.position_ema[0]
        msg.pose.pose.position.y = self.position_ema[1]
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = self.q[1]
        msg.pose.pose.orientation.y = self.q[2]
        msg.pose.pose.orientation.z = self.q[3]
        msg.pose.pose.orientation.w = self.q[0]

        msg.twist.twist.linear.x = self.velocity_ema[0]
        msg.twist.twist.linear.y = self.velocity_ema[1]
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = gyro[0]
        msg.twist.twist.angular.y = gyro[1]
        msg.twist.twist.angular.z = gyro[2]

        self.odom_pub.publish(msg)

        self.get_logger().info(
            f"🛰️ Pos x: {self.position_ema[0]:.2f}, y: {self.position_ema[1]:.2f}, Vel x: {self.velocity_ema[0]:.2f}, y: {self.velocity_ema[1]:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
