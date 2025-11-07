import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TurtlePose

from .map_loader import MapLoader
from .path_planner import PathPlanner

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower')

        self.declare_parameter('map_path', '')
        self.declare_parameter('use_turtlesim', True)
        self.declare_parameter('linear_speed', 0.8)
        self.declare_parameter('angular_gain', 2.0)
        self.declare_parameter('waypoint_tolerance', 0.3)

        self.linear_speed: float = self.get_parameter('linear_speed').value
        self.angular_gain: float = self.get_parameter('angular_gain').value
        self.way_tol: float = self.get_parameter('waypoint_tolerance').value

        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        self.map_loader = MapLoader()
        if not map_path or not self.map_loader.load_map(map_path):
            self.get_logger().error('Карта не загружена. Завершение.')
            raise SystemExit(1)

        self.graph = self.map_loader.get_graph()
        self.planner = PathPlanner(self.graph)

        self.pose: Optional[TurtlePose] = None
        self.sub_pose = self.create_subscription(TurtlePose, '/turtle1/pose', self.cb_pose, 10)
        self.pub_cmd  = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.current_path = None
        self.current_idx = 0
        self.plan_example()
        self.timer = self.create_timer(0.05, self.step)  # 20 Гц
        self.get_logger().info('line_follower запущен')

    def cb_pose(self, msg:TurtlePose):
        self.pose = msg

    def plan_example(self):
        h = self.map_loader.map_data.shape[0]
        start = PathPlanner.find_nearest_node(self.graph, 1, h-2)
        goal  = PathPlanner.find_nearest_node(self.graph, h-2, 1)
        self.current_path = self.planner.dijkstra(start, goal)
        self.current_idx = 0
        if self.current_path:
            self.get_logger().info(f'Путь спланирован: {len(self.current_path)} точек')
        else:
            self.get_logger().warn('Путь не найден')

    def step(self):
        if self.pose is None or not self.current_path or self.current_idx >= len(self.current_path):
            return
        wp = self.current_path[self.current_idx]
        px, py = map(int, wp.split('_'))
        wx, wy = self.map_loader.pixel_to_world(px, py)
        dx = wx - self.pose.x
        dy = wy - self.pose.y
        target_angle = math.atan2(dy, dx)
        angle_err = self.normalize_angle(target_angle - self.pose.theta)
        dist = math.hypot(dx, dy)

        cmd = Twist()
        if dist > self.way_tol:
            cmd.linear.x  = self.linear_speed
            cmd.angular.z = self.angular_gain * angle_err
        else:
            self.current_idx += 1
        self.pub_cmd.publish(cmd)

    @staticmethod
    def normalize_angle(a:float)->float:
        while a > math.pi:  a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

def main():
    rclpy.init()
    try:
        node = LineFollowerNode()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
