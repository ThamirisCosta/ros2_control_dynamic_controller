import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import numpy as np
from scipy.interpolate import CubicSpline
from visualization_msgs.msg import Marker, MarkerArray


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        # Publishers
        self.publisher = self.create_publisher(PoseStamped, '/trajectory_points', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/trajectory_markers', 10)

        # Subscribers
        self.subscription_desired = self.create_subscription(
            PoseStamped, '/desired_position', self.listener_callback, 10)
        self.subscription_current = self.create_subscription(
            PoseStamped, '/current_end_effector_pose', self.update_current_pose, 10)

        # Parameters
        self.num_points = 50  # Número de pontos intermediários na interpolação
        self.tolerance = 0.01  # Tolerância para considerar posição alcançada (1 cm)
        self.publish_rate = 0.05  # Intervalo entre pontos (segundos)

        # State variables
        self.current_pose = None  # Posição atual do end effector [x, y, z]
        self.trajectory_in_progress = False  # Flag para trajetória em progresso

        self.get_logger().info("Interpolador de pontos inicializado.")

    def update_current_pose(self, msg):
        """Atualiza a posição atual do end effector."""
        if msg.header.frame_id != "base_link":
            self.get_logger().warn(
                f"Frame recebido {msg.header.frame_id} diferente do esperado (base_link)")

        self.current_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

    def generate_trajectory(self, start_point, end_point):
        """Gera trajetória usando interpolação cúbica e publica os pontos."""
        if self.trajectory_in_progress:
            self.get_logger().info("Trajetória já em progresso. Ignorando novo comando.")
            return

        self.trajectory_in_progress = True
        self.get_logger().info("Iniciando geração de nova trajetória...")

        try:
            # Interpolação cúbica
            t = np.linspace(0, 1, self.num_points)
            cs = CubicSpline([0, 1], np.vstack([start_point, end_point]), axis=0)
            points = cs(t)

            # Publicar marcadores de trajetória
            self.publish_trajectory_markers(points)

            # Publicar pontos da trajetória
            for point in points:
                if not self.trajectory_in_progress:  # Permite cancelamento
                    break

                pose = self.create_pose_stamped(point)
                self.publisher.publish(pose)
                self.get_clock().sleep_until(
                    self.get_clock().now() + Duration(seconds=self.publish_rate))

        except Exception as e:
            self.get_logger().error(f"Erro na geração de trajetória: {str(e)}")
        finally:
            self.trajectory_in_progress = False
            self.get_logger().info(f"Trajetória finalizada. Posição final: {end_point}")

    def create_pose_stamped(self, position):
        """Cria mensagem PoseStamped a partir de um array de posição."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.w = 1.0  # Orientação neutra (sem rotação)
        return pose

    def publish_trajectory_markers(self, points):
        """Publica marcadores de trajetória no RViz."""
        marker_array = MarkerArray()
        
        for i, point in enumerate(points):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0  # Opacidade total
            marker.color.r = 0.0  # Verde
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = Duration(seconds=30).to_msg()  # Duração do marcador
            marker_array.markers.append(marker)
        
        self.marker_publisher.publish(marker_array)

    def listener_callback(self, msg):
        """Processa nova posição desejada."""
        if self.current_pose is None:
            self.get_logger().warn("Posição atual não disponível - aguardando primeira atualização.")
            return

        end_point = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        distance = np.linalg.norm(self.current_pose - end_point)
        if distance < self.tolerance:
            self.get_logger().info(f"Posição já alcançada (distância: {distance:.4f}m)")
            return

        self.get_logger().info(
            f"Iniciando trajetória de {self.current_pose} para {end_point} "
            f"(distância: {distance:.4f}m)")
        
        self.generate_trajectory(self.current_pose, end_point)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TrajectoryPublisher encerrado por usuário")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
