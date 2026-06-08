#!/usr/bin/env python3
import math
import heapq
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        self.map_frame          = 'map'
        self.robot_frame        = 'base_link'
        self.obstacle_threshold = 50
        self.inflation_radius_m = 0.1
        self.replan_interval    = 2.0
        self.map_data         = None
        self.map_width        = None
        self.map_height       = None
        self.map_resolution   = None
        self.map_origin_x     = None
        self.map_origin_y     = None
        self.inflated_map     = None
        self.current_goal     = None
        self.new_goal_flag    = False
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.timer = self.create_timer(self.replan_interval, self.replan_timer_callback)
        self.get_logger().info('Global Planner Node 시작!')

    def map_callback(self, msg):
        self.map_data       = list(msg.data)
        self.map_width      = msg.info.width
        self.map_height     = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x   = msg.info.origin.position.x
        self.map_origin_y   = msg.info.origin.position.y
        self.inflated_map   = self.build_inflated_map()

    def goal_callback(self, msg):
        self.current_goal  = msg
        self.new_goal_flag = True
        self.get_logger().info(f'새 goal 수신: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        if self.map_data is not None:
            self.plan_path(msg)

    def replan_timer_callback(self):
        if self.new_goal_flag:
            self.new_goal_flag = False
            return
        if self.current_goal is not None and self.map_data is not None:
            self.plan_path(self.current_goal)

    def plan_path(self, goal_msg):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn('로봇 위치 못 가져옴 - TF 확인 필요')
            return
        rx, ry = robot_pose
        gx = goal_msg.pose.position.x
        gy = goal_msg.pose.position.y
        start = self.world_to_grid(rx, ry)
        goal  = self.world_to_grid(gx, gy)
        self.get_logger().info(f'robot=({rx:.2f},{ry:.2f})')
        self.get_logger().info(f'start={start}, goal={goal}')
        self.get_logger().info(f'start obstacle={self.is_obstacle(*start)}')
        if not self.is_valid_grid(*start):
            self.get_logger().warn('시작점이 맵 밖')
            return
        if not self.is_valid_grid(*goal):
            self.get_logger().warn('목표점이 맵 밖')
            return

        goal = self.find_nearest_free(goal)
        if goal is None:
            self.get_logger().warn('목표점 근처 빈 공간 없음')
            return
        path_grids = self.astar(start, goal)
        if path_grids is None:
            self.get_logger().warn('A* 경로 없음')
            return
        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame
        for (grid_x, grid_y) in path_grids:
            wx, wy = self.grid_to_world(grid_x, grid_y)
            pose = PoseStamped()
            pose.header.stamp    = path_msg.header.stamp
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'경로 발행: {len(path_msg.poses)}개 포인트')

    def astar(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score   = {start: 0.0}
        def heuristic(a, b):
            return math.hypot(a[0]-b[0], a[1]-b[1])
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            cx, cy = current
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                nx, ny = cx+dx, cy+dy
                neighbor = (nx, ny)
                if not self.is_valid_grid(nx, ny):
                    continue
                if self.is_obstacle(nx, ny):
                    continue
                if dx != 0 and dy != 0:
                    if self.is_obstacle(cx+dx, cy) or self.is_obstacle(cx, cy+dy):
                        continue
                move_cost   = math.hypot(dx, dy)
                tentative_g = g_score[current] + move_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor]   = tentative_g
                    f = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
        return None

    def build_inflated_map(self):
        if self.map_data is None:
            return None
        inflated = list(self.map_data)
        inflation_cells = max(1, int(self.inflation_radius_m / self.map_resolution))
        for y in range(self.map_height):
            for x in range(self.map_width):
                idx = self.grid_to_index(x, y)
                if self.map_data[idx] >= self.obstacle_threshold:
                    for dy in range(-inflation_cells, inflation_cells+1):
                        for dx in range(-inflation_cells, inflation_cells+1):
                            if math.hypot(dx, dy) <= inflation_cells:
                                nx, ny = x+dx, y+dy
                                if self.is_valid_grid(nx, ny):
                                    nidx = self.grid_to_index(nx, ny)
                                    if inflated[nidx] != -1:
                                        inflated[nidx] = 100
        return inflated

    def is_obstacle(self, gx, gy):
        idx  = self.grid_to_index(gx, gy)
        data = self.inflated_map if self.inflated_map is not None else self.map_data
        val  = data[idx]
        return val >= self.obstacle_threshold

    def find_nearest_free(self, grid_pos, search_radius=10):
        gx, gy = grid_pos
        if not self.is_obstacle(gx, gy):
            return grid_pos
        for r in range(1, search_radius+1):
            for dy in range(-r, r+1):
                for dx in range(-r, r+1):
                    nx, ny = gx+dx, gy+dy
                    if self.is_valid_grid(nx, ny) and not self.is_obstacle(nx, ny):
                        return (nx, ny)
        return None

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn(f'TF 실패: {e}')
            return None

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.map_origin_x) / self.map_resolution)
        gy = int((wy - self.map_origin_y) / self.map_resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        wx = self.map_origin_x + (gx + 0.5) * self.map_resolution
        wy = self.map_origin_y + (gy + 0.5) * self.map_resolution
        return wx, wy

    def is_valid_grid(self, gx, gy):
        return 0 <= gx < self.map_width and 0 <= gy < self.map_height

    def grid_to_index(self, gx, gy):
        return gy * self.map_width + gx


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
