# TODAS LAS FILAS 

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np

class DepthToLaserScan(Node):

    def __init__(self):
        super().__init__('lidar_points')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/kinect/depth/image_raw',
            self.depth_callback,
            10
        )

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # 🔧 Calibración fija (de tu archivo)
        self.fx = 616.0735892347366
        self.cx = 316.3297227951963
        self.range_min = 0.3
        self.range_max = 5.0
        self.frame_id = "laser"

        self.get_logger().info(" KINECT iniciado con todas !")

    def depth_callback(self, msg):
        try:
            # Convertir imagen depth a array NumPy
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            height, width = depth_image.shape

            # Inicializar mensaje LaserScan
            scan_msg = LaserScan()
            scan_msg.header.stamp = msg.header.stamp
            scan_msg.header.frame_id = self.frame_id
            scan_msg.angle_min = -np.arctan((self.cx) / self.fx)
            scan_msg.angle_max = np.arctan((width - self.cx) / self.fx)
            scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / width
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 1.0 / 30.0
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max

            # 🔍 Usar TODAS las filas (promedio vertical por columna)
            depth_mean = np.mean(depth_image, axis=0)  # Promedio vertical para cada columna

            ranges = []
            for i in range(width):
                z = depth_mean[i] / 1000.0  # de mm a metros
                if z == 0.0 or np.isnan(z):
                    ranges.append(float('inf'))
                    continue
                x = (i - self.cx) * z / self.fx
                r = np.sqrt(x**2 + z**2)
                ranges.append(r)

            scan_msg.ranges = ranges
            self.scan_pub.publish(scan_msg)

        except Exception as e:
            self.get_logger().error(f" Error al procesar profundidad: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthToLaserScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
