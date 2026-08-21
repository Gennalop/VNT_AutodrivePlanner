import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from .grid_from_occupancy import grid_from_occupancy_msg
from python_motion_planning.utils import SearchFactory   # antes: from .python_motion_planning.utils import SearchFactory

class PlannerNode(Node):
    def __init__(self):
        super().__init__('lpa_star_planner')

        self.map_msg = None

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)

        self.path_pub = self.create_publisher(Path, '/raw_path', 1)
        self.expand_pub = self.create_publisher(MarkerArray, '/expand_markers', 1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Planner listo. Esperando /map y clic de "2D Goal Pose" en RViz...')
        
    def on_map(self, msg):
        self.map_msg = msg

    def on_goal(self, msg: PoseStamped):
        if self.map_msg is None:
            self.get_logger().warn('Aún no llega /map, no se puede planificar todavía.')
            return
        self.get_logger().info(f'Nuevo goal recibido: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.run_planning(msg.pose.position.x, msg.pose.position.y)

    def get_start_world(self):
        # 'map' debe coincidir con el frame_id que publica tu map_server (Paso 1)
        # 'f1tenth_1' debe coincidir con el base_frame real del robot (confírmalo con tu README de Parte A)
        tf = self.tf_buffer.lookup_transform('map', 'f1tenth_1', Time())
        return tf.transform.translation.x, tf.transform.translation.y

    @staticmethod
    def world_to_grid(x, y, origin, res):
        return (int((x - origin.position.x) / res), int((y - origin.position.y) / res))

    def run_planning(self, gx_world, gy_world):
        env, res, origin = grid_from_occupancy_msg(
        self.map_msg,
        clearance_m=0.25,
        debug_dir=os.path.expanduser('~/autodrive_ws/src/f1tenth_global_planner/output/map_debug'),
    )
        try:
            sx, sy = self.get_start_world()
        except Exception as e:
            self.get_logger().error(f'No se pudo obtener la pose del robot vía TF: {e}')
            return

        start = self.world_to_grid(sx, sy, origin, res)
        goal = self.world_to_grid(gx_world, gy_world, origin, res)
        self.get_logger().info(f'Start (grid): {start} | Goal (grid): {goal}')

        planner = SearchFactory()('lpa_star', start=start, goal=goal, env=env)

        self.run_lpa_star_stepwise(planner, origin, res)

    def run_lpa_star_stepwise(self, planner, origin, res):
        """
        Reimplementa el bucle interno de computeShortestPath() para poder
        publicar planner.EXPAND progresivamente y verlo crecer en RViz.
        """
        prev_len = 0
        while planner.U:
            node = min(planner.U, key=lambda n: n.key)
            if node.key >= planner.calculateKey(planner.goal) and planner.goal.rhs == planner.goal.g:
                break

            planner.U.remove(node)
            planner.EXPAND.append(node)

            if node.g > node.rhs:
                node.g = node.rhs
            else:
                node.g = float('inf')
                planner.updateVertex(node)

            for n in planner.getNeighbor(node):
                planner.updateVertex(n)

            if len(planner.EXPAND) - prev_len >= 15:
                self.publish_expand(planner.EXPAND, origin, res)
                prev_len = len(planner.EXPAND)

        self.publish_expand(planner.EXPAND, origin, res)

        cost, path = planner.extractPath()
        if not path:
            self.get_logger().warn('No se encontró un path válido.')
            return
        self.get_logger().info(f'Path encontrado. Costo: {cost:.2f}, nodos expandidos: {len(planner.EXPAND)}')
        self.publish_path(path, origin, res)

    def publish_expand(self, expand_nodes, origin, res):
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'lpa_star_expand'
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = m.scale.y = 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 0.8
        for n in expand_nodes:
            p = Point()
            p.x = origin.position.x + n.current[0] * res
            p.y = origin.position.y + n.current[1] * res
            m.points.append(p)
        arr.markers.append(m)
        self.expand_pub.publish(arr)

    def publish_path(self, path, origin, res):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for (gx, gy) in path:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = origin.position.x + gx * res
            ps.pose.position.y = origin.position.y + gy * res
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)


def main():
    rclpy.init()
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()