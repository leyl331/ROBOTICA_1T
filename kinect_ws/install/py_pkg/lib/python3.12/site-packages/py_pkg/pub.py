#---------------------------------------------
#    PUBLICAR PROFUNDIDAD, RGB Y CAMERA INFO
#---------------------------------------------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import freenect
import numpy as np

class KinectPublisher(Node):
    def __init__(self):
        super().__init__('kinect_publisher')
        self.bridge = CvBridge()

        # Publicadores activos
        self.rgb_pub = self.create_publisher(Image, '/kinect/rgb', 10)

        
        self.depth_raw_pub = self.create_publisher(Image, '/kinect/depth/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/kinect/depth/camera_info', 10)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("📷 Publicador Kinect ACTIVADO ")

    def timer_callback(self):
        try:
            # Captura RGB
            rgb, _ = freenect.sync_get_video()
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            # Captura profundidad
            depth, _ = freenect.sync_get_depth()
            depth = depth.astype(np.uint16)

            # Publicar imágenes
            self.rgb_pub.publish(self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8'))
            self.depth_raw_pub.publish(self.bridge.cv2_to_imgmsg(depth, encoding='16UC1'))

            # Publicar CameraInfo (estimado para Kinect v1)
            cam_info = CameraInfo()
            cam_info.header.stamp = self.get_clock().now().to_msg()
            cam_info.header.frame_id = "camera_depth_frame"
            cam_info.height = 480
            cam_info.width = 640
            cam_info.distortion_model = "plumb_bob"
            cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            cam_info.k = [525.0, 0.0, 319.5,
                          0.0, 525.0, 239.5,
                          0.0, 0.0, 1.0]
            cam_info.r = [1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0]
            cam_info.p = [525.0, 0.0, 319.5, 0.0,
                          0.0, 525.0, 239.5, 0.0,
                          0.0, 0.0, 1.0, 0.0]
            self.camera_info_pub.publish(cam_info)

        except Exception as e:
            self.get_logger().error(f"❌ Error obteniendo datos: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = KinectPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
