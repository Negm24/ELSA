import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import heapq

# ── MAP ───────────────────────────────────────────────────────────────────────
MAP = {
    "graph": {
        "J1": {
            "J2": {"command": "F", "rooms": ["R101", "R102"]},
            "J3": {"command": "R", "rooms": []}
        },
        "J2": {
            "J1": {"command": "F", "rooms": ["R102", "R101"]},
            "J4": {"command": "L", "rooms": ["R201"]}
        },
        "J3": {
            "J1": {"command": "L", "rooms": []},
            "J4": {"command": "F", "rooms": ["R301", "R302", "R303"]}
        },
        "J4": {
            "J2": {"command": "R", "rooms": ["R201"]},
            "J3": {"command": "F", "rooms": ["R303", "R302", "R301"]}
        }
    },
    "rooms": {
        "R101": {"junction_before": "J1", "junction_after": "J2"},
        "R102": {"junction_before": "J1", "junction_after": "J2"},
        "R201": {"junction_before": "J2", "junction_after": "J4"},
        "R301": {"junction_before": "J3", "junction_after": "J4"},
        "R302": {"junction_before": "J3", "junction_after": "J4"},
        "R303": {"junction_before": "J3", "junction_after": "J4"}
    }
}


class NavigationBrain(Node):

    def __init__(self):
        super().__init__('navigation_brain')

        # ── Parameters (set via ROS2 params or change defaults here) ──────────
        self.declare_parameter('start_node', 'J1')
        self.declare_parameter('target_node', 'R302')

        start_node  = self.get_parameter('start_node').get_parameter_value().string_value
        target_node = self.get_parameter('target_node').get_parameter_value().string_value

        # ── ROS2 I/O ──────────────────────────────────────────────────────────
        self.location_sub = self.create_subscription(
            String, 'detected_sign', self.location_callback, 10)
        self.cmd_pub = self.create_publisher(String, 'robot_topic', 10)

        # ── Map references ────────────────────────────────────────────────────
        self.graph     = MAP["graph"]
        self.rooms_map = MAP["rooms"]
        self.target    = target_node

        # ── Path planning ─────────────────────────────────────────────────────
        self.junction_path, self.target_is_room = self.plan_path(start_node, target_node)

        if not self.junction_path:
            self.get_logger().error(
                f'No path found from {start_node} to {target_node}. Check map.')
            return

        self.get_logger().info(f'MISSION PLANNED: {" → ".join(self.junction_path)}')
        self.get_logger().info(
            f'Target is a {"room" if self.target_is_room else "junction"}: {target_node}')

        # ── State ─────────────────────────────────────────────────────────────
        self.step              = 0
        self.current_leg_rooms = []
        self.rooms_index       = 0
        self.in_final_leg      = False
        self.mission_done      = False

        # ── Kick off ──────────────────────────────────────────────────────────
        self.start_timer = self.create_timer(2.0, self.initiate_movement)

    # ── PATH PLANNING ──────────────────────────────────────────────────────────

    def plan_path(self, start, target):
        """
        Room target  → Dijkstra to junction_before that room, then enter corridor.
        Junction target → Dijkstra directly to it.
        Returns (junction_path, target_is_room).
        """
        if target in self.rooms_map:
            entry_junction = self.rooms_map[target]["junction_before"]
            # Edge case: start IS the entry junction (room is in first corridor)
            if start == entry_junction:
                return [start], True
            return self.dijkstra(start, entry_junction), True
        else:
            return self.dijkstra(start, target), False

    def dijkstra(self, start, target):
        queue   = [(0, start, [start])]
        visited = set()
        while queue:
            cost, node, path = heapq.heappop(queue)
            if node in visited:
                continue
            visited.add(node)
            if node == target:
                return path
            for neighbor in self.graph.get(node, {}):
                if neighbor not in visited:
                    heapq.heappush(queue, (cost + 1, neighbor, path + [neighbor]))
        return []

    # ── STARTUP ────────────────────────────────────────────────────────────────

    def initiate_movement(self):
        self.destroy_timer(self.start_timer)
        self.get_logger().info('Mission start.')

        already_at_entry = (
            self.target_is_room
            and len(self.junction_path) == 1
            and self.step == 0
        )
        if already_at_entry:
            self.enter_final_room_leg()
        else:
            self.update_leg_rooms()
            self.execute_maneuver_from_current()

    # ── SIGN DETECTION ─────────────────────────────────────────────────────────

    def location_callback(self, msg):
        if self.mission_done:
            return

        detected = msg.data.strip()

        if detected in self.rooms_map:
            self.handle_room_sign(detected)
        elif detected in self.graph:
            self.handle_junction_sign(detected)
        else:
            self.get_logger().warn(f'Unknown sign received: "{detected}"')

    def handle_room_sign(self, room):
        # Goal check first
        if self.target_is_room and room == self.target:
            self.get_logger().info(f'TARGET ROOM REACHED: {room}')
            self.stop_robot()
            return

        # Track position along the corridor
        if room in self.current_leg_rooms:
            idx = self.current_leg_rooms.index(room)
            if idx >= self.rooms_index:
                self.rooms_index = idx + 1
                self.get_logger().info(
                    f'Room confirmed: {room} '
                    f'({self.rooms_index}/{len(self.current_leg_rooms)})')
        else:
            self.get_logger().warn(
                f'Unexpected room sign: {room} — not in current leg')

    def handle_junction_sign(self, junction):
        if self.in_final_leg:
            self.get_logger().warn(
                f'Unexpected junction {junction} during final room leg — ignoring')
            return

        if self.step + 1 >= len(self.junction_path):
            self.get_logger().warn(
                f'Unexpected junction {junction} — path already complete')
            return

        expected = self.junction_path[self.step + 1]
        if junction != expected:
            self.get_logger().warn(
                f'Expected junction {expected}, got {junction} — ignoring')
            return

        self.step += 1
        self.get_logger().info(
            f'Junction reached: {junction} '
            f'(step {self.step}/{len(self.junction_path) - 1})')

        at_path_end = (self.step == len(self.junction_path) - 1)

        if at_path_end:
            if self.target_is_room:
                self.enter_final_room_leg()
            else:
                self.get_logger().info(f'TARGET JUNCTION REACHED: {junction}')
                self.stop_robot()
        else:
            self.update_leg_rooms()
            self.execute_maneuver_from_current()

    # ── MOVEMENT ───────────────────────────────────────────────────────────────

    def execute_maneuver_from_current(self):
        current = self.junction_path[self.step]
        next_j  = self.junction_path[self.step + 1]
        command = self.graph[current][next_j]["command"]
        self.get_logger().info(f'{current} → {next_j} : CMD={command}')
        self._dispatch_command(command)

    def enter_final_room_leg(self):
        self.in_final_leg = True
        entry_j = self.junction_path[self.step]
        exit_j  = self.rooms_map[self.target]["junction_after"]
        self.current_leg_rooms = self.graph[entry_j][exit_j].get("rooms", [])
        self.rooms_index = 0
        command = self.graph[entry_j][exit_j]["command"]
        self.get_logger().info(
            f'Entering final corridor {entry_j} → {exit_j}, '
            f'watching for {self.target}')
        self._dispatch_command(command)

    def update_leg_rooms(self):
        if self.step + 1 < len(self.junction_path):
            from_j = self.junction_path[self.step]
            to_j   = self.junction_path[self.step + 1]
            self.current_leg_rooms = self.graph[from_j][to_j].get("rooms", [])
            self.rooms_index = 0
        else:
            self.current_leg_rooms = []
            self.rooms_index = 0

    def _dispatch_command(self, command):
        self.send_cmd(command)
        if command in ('R', 'L'):
            self.turn_timer = self.create_timer(1.0, self._one_shot_forward)

    def _one_shot_forward(self):
        self.send_cmd('F')
        self.destroy_timer(self.turn_timer)

    def send_cmd(self, char):
        msg = String()
        msg.data = char
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'CMD → {char}')

    def stop_robot(self):
        self.mission_done = True
        self.get_logger().info('MISSION COMPLETE.')
        self.send_cmd('S')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationBrain()
    rclpy.spin(node)
    rclpy.shutdown()
