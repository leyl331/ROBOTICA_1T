import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml

class KinectArucoDetector(Node):
    def __init__(self):
        super().__init__('kinect_aruco_detector')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/kinect/rgb',
            self.image_callback,
            10
        )

        self.pose_pub = self.create_publisher(Point, '/marker_pose', 10)
        self.dist_pub = self.create_publisher(String, '/marker_distance_cm', 10)
        self.offset_pub = self.create_publisher(Float32, '/marker_offset_x', 10)  # NUEVO

        # Cargar calibración de cámara
        try:
            with open('/home/leyla/Desktop/calibracion/calibration.yaml', 'r') as f:
                data = yaml.safe_load(f)
                self.camera_matrix = np.array(data['camera_matrix'])
                self.dist_coeffs = np.array(data['dist_coeff'])
                self.get_logger().info("\t Calibración cargada correctamente y ejecucion.")
        except Exception as e:
            self.get_logger().error(f"Error al cargar calibración: {e}")
            self.camera_matrix = np.eye(3)
            self.dist_coeffs = np.zeros((5, 1))

        # Diccionario ArUco
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img_center_x = gray.shape[1] // 2

            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, 0.05, self.camera_matrix, self.dist_coeffs)

                for i in range(len(ids)):
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.03)

                    # Centroide del marcador
                    c = corners[i][0]
                    cx = int(np.mean(c[:, 0]))
                    cy = int(np.mean(c[:, 1]))
                    offset_x = float(cx - img_center_x)

                    tvec = tvecs[i][0]
                    z_cm = tvec[2] * 100  # m a cm

                    # Mostrar sobre imagen
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                    text = f"ID:{ids[i][0]} Dist:{z_cm:.1f}cm"
                    cv2.putText(frame, text, (cx - 50, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Publicar pose 3D
                    point = Point()
                    point.x = round(float(tvec[0]), 2)
                    point.y = round(float(tvec[1]), 2)
                    point.z = round(float(tvec[2]), 2)
                    self.pose_pub.publish(point)

                    # Publicar distancia
                    dist_msg = String()
                    dist_msg.data = f"{z_cm:.2f}"
                    self.dist_pub.publish(dist_msg)

                    # Publicar offset X del centroide
                    offset_msg = Float32()
                    offset_msg.data = offset_x
                    self.offset_pub.publish(offset_msg)

            cv2.imshow("Kinect ArUco View", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = KinectArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
