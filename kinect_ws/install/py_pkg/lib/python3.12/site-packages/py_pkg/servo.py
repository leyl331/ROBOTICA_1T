import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import Servo
from time import sleep

class ServoArucoController(Node):
    def __init__(self):
        super().__init__('servo_aruco_sujetar')

        self.subscriber = self.create_subscription(
            String,
            '/marker_distance_cm',
            self.listener_callback,
            10
        )

        self.servo = Servo(17, min_pulse_width=0.0005, max_pulse_width=0.0025)
        self.servo.value = self.angle_to_value(10)  # Inicia en 10°

        self.get_logger().info('Nodo servo_aruco_controller iniciado')
        self.estado_actual = 'reposo'

    def angle_to_value(self, angle):
        # Convierte un ángulo de 0° a 180° en un valor de -1.0 a 1.0
        return (angle - 90) / 90.0

    def listener_callback(self, msg):
        try:
            distancia_cm = float(msg.data)
            self.get_logger().info(f'Distancia recibida: {distancia_cm:.2f} cm')

            if distancia_cm <= 12.0 and self.estado_actual != 'activo':
                self.get_logger().info('🟢 Distancia corta: moviendo a 80°')
                self.servo.value = self.angle_to_value(80)
                self.estado_actual = 'activo'

            elif distancia_cm > 12.0 and self.estado_actual != 'reposo':
                self.get_logger().info('🔵 Distancia larga: moviendo a 10°')
                self.servo.value = self.angle_to_value(10)
                self.estado_actual = 'reposo'

        except ValueError:
            self.get_logger().warn(' Valor recibido no es válido.')

def main(args=None):
    rclpy.init(args=args)
    node = ServoArucoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
