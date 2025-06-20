import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class FullTFPublisher(Node):
    def __init__(self):
        super().__init__('tf_transf')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # 1. Publicar tf base_link → k_lidar (estática)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'laser'
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.1  # ajusta según tu Kinect
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_tf)
        self.get_logger().info("TF [base_link → laser] publicada (estática)")

        # 2. Suscripción a odometría para publicar odom → base_link
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 3. Publicar tf map → odom (solo si NO usas Cartographer)
        self.enable_map_to_odom = False  # ❌ Desactívalo si usas Cartographer
        if self.enable_map_to_odom:
            self.timer = self.create_timer(1.0, self.publish_map_to_odom)

    def odom_callback(self, msg):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'

        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_map_to_odom(self):
        tf_map_odom = TransformStamped()
        tf_map_odom.header.stamp = self.get_clock().now().to_msg()
        tf_map_odom.header.frame_id = 'map'
        tf_map_odom.child_frame_id = 'odom'

        tf_map_odom.transform.translation.x = 0.0
        tf_map_odom.transform.translation.y = 0.0
        tf_map_odom.transform.translation.z = 0.0
        tf_map_odom.transform.rotation.x = 0.0
        tf_map_odom.transform.rotation.y = 0.0
        tf_map_odom.transform.rotation.z = 0.0
        tf_map_odom.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf_map_odom)

def main():
    rclpy.init()
    node = FullTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
