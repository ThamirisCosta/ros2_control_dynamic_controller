import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class DesiredPositionPublisher(Node):
    def __init__(self):
        super().__init__('desired_position_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/desired_position', 10)
        self.timer = None  # Será inicializado após o delay
        
        # Adicionando parâmetros configuráveis
        self.declare_parameters(
            namespace='',
            parameters=[
                ('initial_delay', 5.0),
                ('publish_rate', 10.0),
                ('x_position', 1.391),
                ('y_position', 0.01),
                ('z_position', -0.004)
            ]
        )
        
        # Timer para o delay inicial
        self.initial_delay_timer = self.create_timer(
            self.get_parameter('initial_delay').value, 
            self.start_publishing
        )
        
        self.get_logger().info("Desired position publisher initialized with delay of {} seconds".format(
            self.get_parameter('initial_delay').value))

    def start_publishing(self):
        # Configura o timer de publicação após o delay inicial
        if self.timer is None:
            rate = self.get_parameter('publish_rate').value
            self.timer = self.create_timer(1.0/rate, self.publish_desired_position)
            self.destroy_timer(self.initial_delay_timer)
            self.get_logger().info("Started publishing desired positions at {} Hz".format(rate))

    def publish_desired_position(self):
        msg = PoseStamped()
        
        # Usando parâmetros para as posições
        msg.pose.position.x = self.get_parameter('x_position').value
        msg.pose.position.y = self.get_parameter('y_position').value
        msg.pose.position.z = self.get_parameter('z_position').value
        
        # Orientação neutra
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0  # W deve ser 1 para orientação neutra

        # Timestamp e frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        self.publisher_.publish(msg)
        self.get_logger().debug('Publishing desired position', throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = DesiredPositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
