#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
import numpy as np


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('min_frontier_size', 10)
        self.min_frontier_size = self.get_parameter('min_frontier_size').value

        self.map_data = None
        self.map_info = None
        self.exploring = False
        self.failed_goals = []
        self.current_goal = None
        self.goal_start_time = None

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/frontier_markers', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.timer = self.create_timer(5.0, self.explore)
        self.watchdog = self.create_timer(2.0, self.check_watchdog)
        self.get_logger().info('Frontier Explorer started.')

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        self.map_info = msg.info

    def check_watchdog(self):
        """If a goal is taking too long or finished instantly, reset."""
        if self.exploring and self.goal_start_time:
            elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
            if elapsed > 60.0:
                self.get_logger().warn('Goal timeout — resetting')
                self.exploring = False

    def explore(self):
        if self.map_data is None:
            self.get_logger().info('No map yet...')
            return
        if self.exploring:
            return

        frontiers = self.find_frontiers()
        if not frontiers:
            self.get_logger().info('No frontiers — exploration complete!')
            return

        self.publish_markers(frontiers)

        # Get robot position from map
        robot_col, robot_row = self.world_to_cell(0.0, 0.0)

        candidates = []
        for cluster in frontiers:
            if len(cluster) < self.min_frontier_size:
                continue
            rows = [c[0] for c in cluster]
            cols = [c[1] for c in cluster]
            cr = int(np.mean(rows))
            cc = int(np.mean(cols))

            if not self.is_safe_cell(cr, cc):
                continue

            cx, cy = self.cell_to_world(cr, cc)

            # Skip goals too close to robot
            dist_to_robot = np.sqrt((cr - robot_row)**2 + (cc - robot_col)**2)
            if dist_to_robot * self.map_info.resolution < 0.3:
                continue

            # Skip previously failed goals
            skip = False
            for fx, fy in self.failed_goals:
                if np.sqrt((cx - fx)**2 + (cy - fy)**2) < 0.4:
                    skip = True
                    break
            if skip:
                continue

            candidates.append((len(cluster), cx, cy, cr, cc))

        if not candidates:
            self.get_logger().warn('All frontiers failed before — clearing memory')
            self.failed_goals = []
            return

        # Pick largest frontier
        candidates.sort(reverse=True)
        _, wx, wy, gr, gc = candidates[0]

        self.current_goal = (wx, wy)
        self.get_logger().info(f'Sending goal: ({wx:.2f}, {wy:.2f}), frontier size: {candidates[0][0]}')
        self.send_nav_goal(wx, wy)

    def world_to_cell(self, x, y):
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        col = int((x - ox) / res)
        row = int((y - oy) / res)
        return col, row

    def is_safe_cell(self, row, col):
        h, w = self.map_data.shape
        margin = 4
        r0 = max(0, row - margin)
        r1 = min(h, row + margin)
        c0 = max(0, col - margin)
        c1 = min(w, col + margin)
        region = self.map_data[r0:r1, c0:c1]
        if np.any(region > 50):
            return False
        if np.sum(region == -1) > (margin * margin):
            return False
        return True

    def find_frontiers(self):
        h, w = self.map_data.shape
        frontier_cells = []

        for r in range(1, h - 1):
            for c in range(1, w - 1):
                if self.map_data[r, c] != 0:
                    continue
                neighbors = self.map_data[r-1:r+2, c-1:c+2].flatten()
                if -1 in neighbors:
                    frontier_cells.append((r, c))

        clusters = []
        cell_set = set(frontier_cells)
        visited_set = set()

        for cell in frontier_cells:
            if cell in visited_set:
                continue
            cluster = []
            queue = [cell]
            while queue:
                cur = queue.pop()
                if cur in visited_set:
                    continue
                visited_set.add(cur)
                cluster.append(cur)
                r, c = cur
                for nr, nc in [(r-1,c),(r+1,c),(r,c-1),(r,c+1)]:
                    if (nr, nc) in cell_set and (nr, nc) not in visited_set:
                        queue.append((nr, nc))
            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)

        return clusters

    def cell_to_world(self, row, col):
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        x = col * res + ox + res / 2.0
        y = row * res + oy + res / 2.0
        return x, y

    def send_nav_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.exploring = True
        self.goal_start_time = self.get_clock().now()
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal REJECTED — blacklisting')
            if self.current_goal:
                self.failed_goals.append(self.current_goal)
            self.exploring = False
            return
        self.get_logger().info('Goal accepted')
        handle.get_result_async().add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        status = future.result().status
        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Goal SUCCEEDED in {elapsed:.1f}s')
            if elapsed < 2.0:
                # Finished too fast = robot was already there, blacklist it
                self.get_logger().warn('Finished too fast — blacklisting as invalid')
                if self.current_goal:
                    self.failed_goals.append(self.current_goal)
        else:
            self.get_logger().warn(f'Goal FAILED status={status} in {elapsed:.1f}s — blacklisting')
            if self.current_goal:
                self.failed_goals.append(self.current_goal)
        self.exploring = False

    def feedback_cb(self, feedback):
        pass

    def publish_markers(self, clusters):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, cluster in enumerate(clusters):
            rows = [c[0] for c in cluster]
            cols = [c[1] for c in cluster]
            cx, cy = self.cell_to_world(int(np.mean(rows)), int(np.mean(cols)))
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.r = 1.0
            m.color.g = 0.4
            m.color.b = 0.0
            m.color.a = 0.9
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
