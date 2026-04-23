import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import heapq

# YOUR BUILDING MAP — edit this to match your real building
GRAPH = {
    'start':      {'junction_a': 1},
    'junction_a': {'start': 1, 'room_101': 1, 'junction_b': 1},
    'junction_b': {'junction_a': 1, 'room_102': 1, 'junction_c': 1},
    'junction_c': {'junction_b': 1, 'room_103': 1},
    'room_101':   {'junction_a': 1},
    'room_102':   {'junction_b': 1},
    'room_103':   {'junction_c': 1},
}

# DIRECTIONS — what to do when moving from node A to node B
DIRECTIONS = {
    ('start', 'junction_a'):      'F',
    ('junction_a', 'room_101'):   'R',
    ('junction_a', 'junction_b'): 'F',
    ('junction_b', 'room_102'):   'R',
    ('junction_b', 'junction_c'): 'F',
    ('junction_c', 'room_103'):   'R',
}

def dijkstra(graph, start, target):
    queue = [(0, start, [start])]
    visited = set()
    while queue:
        cost, node, path = heapq.heappop(queue)
        if node in visited:
            continue
        visited.add(node)
        if node == target:
            return path
        for neighbor, weight in graph[node].items():
            if neighbor not in visited:
                heapq.heappush(queue, (cost + weight, neighbor, path + [neighbor]))
    return []

class NavigationBrain(Node):
    def __init__(self):
        super().__init__('navigation_brain')

        # Subscribes to campus_vision for current location
        self.location_sub = self.create_subscription(
            String, 'detected_room', self.location_callback, 10)

        # Publishes movement commands to micro_ros_agent
        self.cmd_pub = self.create_publisher(String, 'robot_topic', 10)

        self.current_node = 'start'
        self.target_node = 'room_103'  # Change this to your target
        self.path = dijkstra(GRAPH, self.current_node, self.target_node)
        self.step = 1  # Already at 'start', waiting for first junction
        
        self.get_logger().info(f'Path planned: {self.path}')
        self.move_next()

    def location_callback(self, msg):
        if self.step >= len(self.path):
            return  # Already at target (for list index out of range problem)
        detected = msg.data.lower().replace(' ', '_')
        if detected == self.path[self.step]:
            self.get_logger().info(f'Confirmed at: {detected}')
            self.current_node = detected
            self.step += 1
            if self.step >= len(self.path):
                self.stop_robot()
            else:
                self.move_next()

    def move_next(self):
        if self.step >= len(self.path):
            return
        current = self.path[self.step - 1]
        next_node = self.path[self.step]
        direction = DIRECTIONS.get((current, next_node), 'F')
        self.get_logger().info(f'Moving: {direction} toward {next_node}')

        msg = String()

        if direction in ['R', 'L']:
            # First turn
            msg.data = direction
            self.cmd_pub.publish(msg)
            # Then after 1 second, drive forward toward the room
            self.create_timer(1.0, lambda: self.send_forward())
        else:
            msg.data = 'F'
            self.cmd_pub.publish(msg)

    def send_forward(self):
        msg = String()
        msg.data = 'F'
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.get_logger().info(f'Arrived at {self.target_node}!')
        msg = String()
        msg.data = 'S'  # Stop command for micro_ros_agent
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationBrain()
    rclpy.spin(node)
    rclpy.shutdown()