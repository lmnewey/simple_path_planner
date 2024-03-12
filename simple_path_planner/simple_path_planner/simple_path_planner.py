import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from math import hypot, atan2, pi
from nav_msgs.msg import Odometry

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.map = None
        self.waypoints = []
        self.mission_received = False
        self.current_pose = None

        # Subscribe to the odometry topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # Subscribe to the map topic (if available)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)

        # Subscribe to the mission topic
        self.mission_subscription = self.create_subscription(
            PoseStamped,
            'mission',
            self.mission_callback,
            10)

        # Advertise the waypoints topic
        self.waypoints_publisher = self.create_publisher(
            PoseStamped,
            'waypoints',
            10)

        # Set parameters for waypoint distance and angle
        self.declare_parameter('waypoint_distance', 1.0)
        self.declare_parameter('waypoint_angle', pi/4)
  
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        #self.get_logger().info('Current Pose: %s', str(self.current_pose))
        

    def map_callback(self, msg):
        self.map = msg

    def mission_callback(self, msg):
        self.mission_received = True
        self.plan_path(msg.pose)

    def plan_path(self, goal_pose):
        self.get_logger().info('Planning path in free space')
        start_pose = self.current_pose
        self.get_logger().info('Got Current Pose: %s' % str(start_pose))
        path = self.generate_path(start_pose, goal_pose)
        self.get_logger().info('Path Planned: %s' % str(path))
        self.waypoints = []
        for pose in path:
            waypoint = PoseStamped()
            waypoint.pose = pose
            self.waypoints.append(waypoint)
        self.publish_waypoints()

    def a_star(self, start_pose, goal_pose):
        pass

    def get_lowest_f_score_node(self, f_scores, open_set):
        lowest_f_score = float('inf')
        lowest_node = None
        for node in open_set:
            if f_scores[node] < lowest_f_score:
                lowest_f_score = f_scores[node]
                lowest_node = node
        return lowest_node
    
    def generate_path(self, start_pose, goal_pose):
        start_node = self.create_node(start_pose)
        goal_node = self.create_node(goal_pose)
        open_set = [start_node]
        came_from = {}
        g_score = {start_node: 0}
        f_score = {start_node: self.heuristic(start_node, goal_node)}
        self.get_logger().info('Generating Path')
        while open_set:
            print(len(open_set))
            current_node = self.get_lowest_f_score_node(f_score, open_set)
            if current_node == goal_node:
                path = self.reconstruct_path(came_from, current_node)
                self.get_logger().info('Found a valid Path')
                return path
            open_set.remove(current_node)
            neighbors = self.get_neighbors(current_node)
            for neighbor in neighbors:
                tentative_g_score = g_score[current_node] + self.distance(current_node, neighbor)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current_node
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal_node)
                    if neighbor not in open_set:
                        open_set.append(neighbor)
        self.get_logger().info('Failed to find Path')
        return []

    def create_node(self, pose):
        return (pose.position.x, pose.position.y)

    def get_neighbors(self, node):
        neighbors = []
        dxdy = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        for dx, dy in dxdy:
            new_node = (node[0] + dx, node[1] + dy)
            neighbors.append(new_node)
        return neighbors

    def distance(self, node1, node2):
        dx = node1[0] - node2[0]
        dy = node1[1] - node2[1]
        return math.sqrt(dx**2 + dy**2)

    def heuristic(self, node1, node2):
        return self.distance(node1, node2)

    def reconstruct_path(self, came_from, current_node):
        path = []
        while current_node in came_from:
            pose = PoseStamped()
            pose.position.x = current_node[0]
            pose.position.y = current_node[1]
            path.append(pose)
            current_node = came_from[current_node]
        return path[::-1]

    def generate_waypoints(self, path):
        waypoint_distance = self.get_parameter('waypoint_distance').value
        waypoint_angle = self.get_parameter('waypoint_angle').value
        waypoints = [path[0]]
        for i in range(1, len(path)):
            prev_pose = path[i-1]
            cur_pose = path[i]
            dist = hypot(cur_pose.position.x - prev_pose.position.x, cur_pose.position.y - prev_pose.position.y)
            angle = atan2(cur_pose.position.y - prev_pose.position.y, cur_pose.position.x - prev_pose.position.x)
            if dist > waypoint_distance or abs(angle) > waypoint_angle:
                waypoints.append(cur_pose)
        return waypoints

    def publish_waypoints(self):
        for waypoint in self.waypoints:
            self.waypoints_publisher.publish(waypoint)
        self.mission_received = False

def main(args=None):
    rclpy.init(args=args)

    path_planner = PathPlanner()

    rclpy.spin(path_planner)

    path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# import math
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import OccupancyGrid
# from math import hypot, atan2, pi
# from nav_msgs.msg import Odometry

# class PathPlanner(Node):
#     def __init__(self):
#         super().__init__('path_planner')
#         self.map = None
#         self.waypoints = []
#         self.mission_received = False
#         self.current_pose = None

#         # Subscribe to the odometry topic
#         self.odom_subscription = self.create_subscription(
#             Odometry,
#             'odom',
#             self.odom_callback,
#             10)
#         # Subscribe to the map topic (if available)
#         self.map_subscription = self.create_subscription(
#             OccupancyGrid,
#             'map',
#             self.map_callback,
#             10)

#         # Subscribe to the mission topic
#         self.mission_subscription = self.create_subscription(
#             PoseStamped,
#             'mission',
#             self.mission_callback,
#             10)

#         # Advertise the waypoints topic
#         self.waypoints_publisher = self.create_publisher(
#             PoseStamped,
#             'waypoints',
#             10)

#         # Set parameters for waypoint distance and angle
#         self.declare_parameter('waypoint_distance', 1.0)
#         self.declare_parameter('waypoint_angle', pi/4)
  
#     def odom_callback(self, msg):
#         self.current_pose = msg.pose.pose


#     def map_callback(self, msg):
#         self.map = msg

#     def mission_callback(self, msg):
#         self.mission_received = True
#         self.plan_path(msg.pose)

#     def plan_path(self, goal_pose):
#         self.get_logger().info('Planning path in free space')
#         start_pose = self.current_pose#self.get_current_pose()
#         self.get_logger().info('Got Current Pose')
#         path = self.generate_path(start_pose, goal_pose)
#         self.get_logger().info('Path Planned')
#         self.waypoints = []
#         for pose in path:
#             waypoint = PoseStamped()
#             waypoint.pose = pose
#             self.waypoints.append(waypoint)
#         self.publish_waypoints()

#     def a_star(self, start_pose, goal_pose):
#         # Implement A* algorithm here to find a path from start_pose to goal_pose
#         # Return a list of poses representing the path
#         pass
#     def get_lowest_f_score_node(self, f_scores, open_set):
#         lowest_f_score = float('inf')
#         lowest_node = None
#         for node in open_set:
#             if f_scores[node] < lowest_f_score:
#                 lowest_f_score = f_scores[node]
#                 lowest_node = node
#         return lowest_node
    
#     def generate_path(self, start_pose, goal_pose):
#         start_node = self.create_node(start_pose)
#         goal_node = self.create_node(goal_pose)
#         open_set = [start_node]
#         came_from = {}
#         g_score = {start_node: 0}
#         f_score = {start_node: self.heuristic(start_node, goal_node)}
#         self.get_logger().info('Generating Path')
#         while open_set:
#             current_node = self.get_lowest_f_score_node(f_score, open_set)
#             if current_node == goal_node:
#                 path = self.reconstruct_path(came_from, current_node)
#                 return path
#             open_set.remove(current_node)
#             neighbors = self.get_neighbors(current_node)
#             for neighbor in neighbors:
#                 tentative_g_score = g_score[current_node] + self.distance(current_node, neighbor)
#                 if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
#                     came_from[neighbor] = current_node
#                     g_score[neighbor] = tentative_g_score
#                     f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal_node)
#                     if neighbor not in open_set:
#                         open_set.append(neighbor)

#         return []

#     def create_node(self, pose):
#         return (pose.position.x, pose.position.y)

#     def get_neighbors(self, node):
#         neighbors = []
#         dxdy = [(0, 1), (1, 0), (0, -1), (-1, 0)]
#         for dx, dy in dxdy:
#             new_node = (node[0] + dx, node[1] + dy)
#             neighbors.append(new_node)
#         return neighbors

#     def distance(self, node1, node2):
#         dx = node1[0] - node2[0]
#         dy = node1[1] - node2[1]
#         return math.sqrt(dx**2 + dy**2)

#     def heuristic(self, node1, node2):
#         return self.distance(node1, node2)

#     def reconstruct_path(self, came_from, current_node):
#         path = []
#         while current_node in came_from:
#             pose = PoseStamped()
#             pose.position.x = current_node[0]
#             pose.position.y = current_node[1]
#             path.append(pose)
#             current_node = came_from[current_node]
#         return path[::-1]

#     def generate_waypoints(self, path):
#         waypoint_distance = self.get_parameter('waypoint_distance').value
#         waypoint_angle = self.get_parameter('waypoint_angle').value
#         waypoints = [path[0]]
#         for i in range(1, len(path)):
#             prev_pose = path[i-1]
#             cur_pose = path[i]
#             dist = hypot(cur_pose.position.x - prev_pose.position.x, cur_pose.position.y - prev_pose.position.y)
#             angle = atan2(cur_pose.position.y - prev_pose.position.y, cur_pose.position.x - prev_pose.position.x)
#             if dist > waypoint_distance or abs(angle) > waypoint_angle:
#                 waypoints.append(cur_pose)
#         return waypoints

#     def publish_waypoints(self):
#         for waypoint in self.waypoints:
#             self.waypoints_publisher.publish(waypoint)
#         self.mission_received = False

# def main(args=None):
#     rclpy.init(args=args)

#     path_planner = PathPlanner()

#     rclpy.spin(path_planner)

#     path_planner.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
