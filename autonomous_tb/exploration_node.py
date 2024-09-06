import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import IsPathValid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.msg import GoalStatus
from rclpy.task import Future
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.distance import cdist
import asyncio
import threading

class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')
        
        # Create separate callback groups
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = ReentrantCallbackGroup()
        self.map_callback_group = MutuallyExclusiveCallbackGroup()
        
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            5,
            callback_group=self.map_callback_group)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            20)
        self.amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_pose_callback,
            10)
        self.frontier_publisher = self.create_publisher(MarkerArray, 'frontiers', 30)
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10)
        # self.map_debug_publisher = self.create_publisher(OccupancyGrid, 'map_debug', 10)
        
        self.is_path_valid_client = self.create_client(
            IsPathValid, 
            'is_path_valid',
            callback_group=self.service_group
        )
        while not self.is_path_valid_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IsPathValid service not available, waiting...')
        
        self.current_goal = None
        self.map = None
        self.initial_pose_sent = False
        self.latest_odom = None
        self.latest_amcl_pose = None
        self.goal_handle = None
        self.cluster_tolerance = 0.10  # meters
        self.min_frontier_size = 5  # minimum number of points to form a cluster
        self.obstacle_clearance = 0.2  # meters
        self.min_goal_distance = 0.3  # minimum distance for a new goal
        self.exploration_initialized = False
        self.start_position = None
        self.navigation_in_progress = False

        self.get_logger().info('Exploration node initialized')

        # Add a timer to periodically check status
        self.create_timer(5.0, self.check_status)
        
        # Goal planning timer
        self.planning_timer = self.create_timer(
            5.0,  # Plan every 5 seconds, adjust as needed
            self.planning_timer_callback,
            callback_group=self.timer_group
        )

        # Create an asyncio event loop for this node
        self.loop = asyncio.get_event_loop()

    def check_status(self):
        self.get_logger().info(f'Current status: Map: {"Received" if self.map else "Not received"}, '
                               f'Current goal: {self.current_goal if self.current_goal else "None"}, '
                               f'Initial pose sent: {self.initial_pose_sent}, '
                               f'Latest odom received: {"Yes" if self.latest_odom else "No"} ')

    def map_callback(self, msg):
        # self.get_logger().info(f'Received map update. Size: {msg.info.width}x{msg.info.height}')
        self.map = msg
        if not self.initial_pose_sent:
            self.send_initial_pose()
        elif not self.exploration_initialized:
            self.initialize_exploration()
        elif self.current_goal is None and not self.navigation_in_progress:
            asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)

    def initialize_exploration(self):
        if self.latest_odom is None:
            self.get_logger().warn('No odometry data available. Cannot initialize exploration.')
            return False
        
        self.start_position = (self.latest_odom.pose.pose.position.x, self.latest_odom.pose.pose.position.y)
        self.exploration_initialized = True
        self.get_logger().info(f'Exploration initialized. Start position: {self.start_position}')
        asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)
        return True

    def planning_timer_callback(self):
        if not self.exploration_initialized:
            if self.initialize_exploration():
                self.get_logger().info('Exploration initialized in timer callback')
            else:
                self.get_logger().warn('Failed to initialize exploration in timer callback')
            return

        if self.map is None:
            self.get_logger().warn('No map available for planning')
            return
        
        if self.current_goal is None:
            asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)
        else:
            self.check_and_replan()

    def check_and_replan(self):
        if self.latest_odom is None or self.current_goal is None:
            return
        
        robot_pos = np.array([self.latest_odom.pose.pose.position.x, self.latest_odom.pose.pose.position.y])
        goal_pos = np.array(self.current_goal)
        distance_to_goal = np.linalg.norm(robot_pos - goal_pos)
        
        if distance_to_goal < 0.25:  # If within 0.2 meters of the goal
            self.get_logger().info('Near current goal. Planning next goal.')
            self.navigation_in_progress = False
            self.current_goal = None
            asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)
        else:
            asyncio.run_coroutine_threadsafe(self.check_path_and_replan(robot_pos, goal_pos), self.loop)

    async def check_path_and_replan(self, robot_pos, goal_pos):
        if not await self.is_path_free_nav2(robot_pos, goal_pos):
            self.get_logger().info('Path to current goal may be obstructed. Replanning.')
            self.current_goal = None
            self.navigation_in_progress = False
            await self.plan_next_goal()

    async def is_path_free_nav2(self, start, goal):
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose.position.x = start[0]
        start_pose.pose.position.y = start[1]
        start_pose.pose.orientation.w = 1.0

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.orientation.w = 1.0

        request = IsPathValid.Request()
        request.path.header.frame_id = 'map'
        request.path.header.stamp = self.get_clock().now().to_msg()
        request.path.poses = [start_pose, goal_pose]

        future = self.is_path_valid_client.call_async(request)
        try:
            # Wait for the result using asyncio
            response = await asyncio.wait_for(future, timeout=1.0)
            return response.is_valid
        except asyncio.TimeoutError:
            self.get_logger().warn('Service call timed out')
            return False
        except Exception as e:
            self.get_logger().warn(f'Service call failed: {e}')
            return False

    def odom_callback(self, msg):
        self.latest_odom = msg
        self.get_logger().debug(f'Received odom update. Position: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})')

    def amcl_pose_callback(self, msg):
        self.latest_amcl_pose = msg
        self.get_logger().debug(f'Received AMCL pose update. Position: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})')

    def send_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'

        if self.latest_odom:
            # Use odometry data if available
            initial_pose.pose.pose = self.latest_odom.pose.pose
            self.get_logger().info('Sent initial pose estimate based on odometry')
        else:
            # Set to (0,0,0) if no odometry data
            initial_pose.pose.pose.position.x = 0.0
            initial_pose.pose.pose.position.y = 0.0
            initial_pose.pose.pose.position.z = 0.0
            initial_pose.pose.pose.orientation.x = 0.0
            initial_pose.pose.pose.orientation.y = 0.0
            initial_pose.pose.pose.orientation.z = 0.0
            initial_pose.pose.pose.orientation.w = 1.0
            self.get_logger().info('Sent initial pose estimate at (0, 0, 0)')

        # Set covariance (adjust as needed)
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

        self.initial_pose_publisher.publish(initial_pose)
        self.initial_pose_sent = True
        if not self.exploration_initialized and self.map is not None:
            self.initialize_exploration()

    async def plan_next_goal(self):
        if self.navigation_in_progress and self.current_goal is not None:
            self.get_logger().info('Navigation already in progress, skipping planning')
            return
        if self.map is None:
            self.get_logger().warn('No map available for planning')
            return

        frontiers = self.detect_frontiers()
        
        if not frontiers:
            self.get_logger().warn('No frontiers detected')
            return

        map_data = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))

        clustered_frontiers = self.cluster_frontiers(frontiers)
        filtered_frontiers = self.filter_obstacles(clustered_frontiers, map_data)

        if not filtered_frontiers:
            self.get_logger().warn('No valid frontiers after filtering')
            return
        else:
            self.get_logger().info(f'Detected {len(filtered_frontiers)} filtered frontiers')

        sorted_frontiers = self.select_best_frontier(filtered_frontiers)
        self.visualize_frontiers(sorted_frontiers)

        if self.latest_odom is None:
            self.get_logger().warn('No odometry data available')
            return

        robot_pos = np.array([self.latest_odom.pose.pose.position.x, self.latest_odom.pose.pose.position.y])

        for frontier in sorted_frontiers:
            frontier_arr = np.array(frontier)
            if await self.is_path_free_nav2(robot_pos, frontier):
                if not self.navigation_in_progress or self.current_goal is None:
                    self.get_logger().info(f'Selected new goal: ({frontier[0]}, {frontier[1]})')
                    self.current_goal = frontier
                    self.send_goal(frontier[0], frontier[1])
                return

        self.get_logger().warn('No reachable frontiers found')
        await self.handle_no_reachable_frontiers()

    def select_best_frontier(self, frontiers):
        if not frontiers or self.latest_odom is None:
            return None
        robot_pos = np.array([self.latest_odom.pose.pose.position.x, self.latest_odom.pose.pose.position.y])
        # distances = cdist([robot_pos], frontiers, metric='euclidean')[0]
        distances = [np.linalg.norm(robot_pos-np.array(frontier)) for frontier in frontiers]
        frontier_distances = list(zip(frontiers, distances))
        valid_frontier_distances = [(f, d) for f, d in frontier_distances if d >= self.min_goal_distance]
        if not valid_frontier_distances:
            self.get_logger().warn('No frontiers beyond minimum distance')
            return None
        sorted_frontiers = sorted(valid_frontier_distances, key=lambda x: x[1])
        return [f for f, _ in sorted_frontiers]

    def detect_frontiers(self):
        if self.map is None:
            return []

        frontiers = []
        map_data = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        
        unknown_space = -1
        occupied_threshold = 90

        kernel = np.array([
            [1, 1, 1],
            [1, 0, 1],
            [1, 1, 1]
        ])

        padded_map = np.pad(map_data, pad_width=1, mode='constant', constant_values=unknown_space)

        for y in range(1, padded_map.shape[0] - 1):
            for x in range(1, padded_map.shape[1] - 1):
                if padded_map[y, x] != unknown_space and padded_map[y, x] < occupied_threshold:
                    neighborhood = padded_map[y-1:y+2, x-1:x+2]
                    if np.any(neighborhood[kernel == 1] == unknown_space):
                        frontiers.append((x-1, y-1))

        world_frontiers = []
        for x, y in frontiers:
            world_x = x * self.map.info.resolution + self.map.info.origin.position.x
            world_y = y * self.map.info.resolution + self.map.info.origin.position.y
            world_frontiers.append((world_x, world_y))

        # self.get_logger().info(f'Detected {len(world_frontiers)} frontiers')
        return world_frontiers

    def cluster_frontiers(self, frontiers):
        if len(frontiers) < 2:
            return frontiers

        frontier_array = np.array(frontiers)
        db = DBSCAN(eps=self.cluster_tolerance, min_samples=self.min_frontier_size).fit(frontier_array)
        
        labels = db.labels_
        unique_labels = set(labels)

        clustered_frontiers = []
        for label in unique_labels:
            if label == -1:  # Noise points
                continue
            class_member_mask = (labels == label)
            cluster_points = frontier_array[class_member_mask]
            centroid = np.mean(cluster_points, axis=0)
            clustered_frontiers.append(centroid)

        return clustered_frontiers

    def filter_obstacles(self, frontiers, map_data):
        filtered_frontiers = []
        for frontier in frontiers:
            x, y = frontier
            grid_x = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
            grid_y = int((y - self.map.info.origin.position.y) / self.map.info.resolution)

            clearance_cells = int(self.obstacle_clearance / self.map.info.resolution)
            for dx in range(-clearance_cells, clearance_cells + 1):
                for dy in range(-clearance_cells, clearance_cells + 1):
                    if dx*dx + dy*dy <= clearance_cells*clearance_cells:
                        check_x = grid_x + dx
                        check_y = grid_y + dy
                        if 0 <= check_x < self.map.info.width and 0 <= check_y < self.map.info.height:
                            if map_data[check_y, check_x] > 50:  # Assuming > 50 is obstacle
                                break
                else:
                    continue
                break
            else:
                filtered_frontiers.append(frontier)

        return filtered_frontiers
 
    def visualize_frontiers(self, frontiers):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "frontiers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        
        self.frontier_publisher.publish(marker_array)

    def send_goal(self, x, y):
        self.current_goal = (x, y)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info('Waiting for NavigateToPose action server...')
        self.navigate_to_pose_client.wait_for_server()
        self.get_logger().info(f'Sending goal request to ({x}, {y})...')
        self.send_goal_future = self.navigate_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.navigation_in_progress = True
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.current_goal = None
            self.navigation_in_progress = False
            asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)
            return

        self.get_logger().info('Goal accepted')
        self.goal_handle = goal_handle

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            self.navigation_in_progress = False
            self.current_goal = None
            asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            self.handle_navigation_failure()

    def handle_navigation_failure(self):
        self.get_logger().warn('Navigation to current goal failed. Clearing current goal and replanning.')
        self.current_goal = None
        self.navigation_in_progress = False
        asyncio.run_coroutine_threadsafe(self.plan_next_goal(), self.loop)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Navigation feedback received: {feedback}')

    async def handle_no_reachable_frontiers(self):
        self.get_logger().warn('No reachable frontiers found. Returning to start position.')
        if self.start_position:
            robot_pos = (self.latest_odom.pose.pose.position.x, self.latest_odom.pose.pose.position.y)
            if await self.is_path_free_nav2(robot_pos, self.start_position):
                self.get_logger().info(f'Start: ({self.start_position[0]}, {self.start_position[1]})')
                self.send_goal(0.00, 0.00)
            else:
                self.get_logger().warn('No Path to Start Position. Unable to return.')
                self.get_logger().info(f'Start: ({self.start_position[0]}, {self.start_position[1]})')
        else:
            self.get_logger().error('Start position unknown. Unable to return.')

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Run the executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Run the asyncio event loop in the main thread
    try:
        node.loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.loop.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()