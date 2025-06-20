# ------------------------------------
# recibe distancia (String) y publica cmd_vel
# ------------------------------------
#sssssssssssss


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MarkerFollower(Node):
    def __init__(self):
        super().__init__('velocidad_aruco')

        self.subscription = self.create_subscription(
            String,  # CORREGIDO: mensaje tipo String
            '/marker_distance_cm',
            self.marker_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("🚶 Nodo MarkerFollower activo")

    def marker_callback(self, msg):
        try:
            distancia_z = float(msg.data) / 100.0  # cm → metros
            velocidad = Twist()

            if distancia_z > 0.35:
                velocidad.linear.x = 0.1
            elif distancia_z >= 0.30:
                velocidad.linear.x = 0.05
            else:
                velocidad.linear.x = 0.0

            velocidad.angular.z = 0.0
            self.cmd_pub.publish(velocidad)
            self.get_logger().info(f"📏 Distancia: {distancia_z:.2f} m → vel.x: {velocidad.linear.x:.2f}")
        except Exception as e:
            self.get_logger().error(f"Error al convertir distancia: {e}")

def main(args=None):
    rclpy.init(args=args)
    nodo = MarkerFollower()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
