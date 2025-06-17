#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose,  Point, Quaternion
from std_msgs.msg import Bool
import math
import ast  # Safe string-to-python list conversion

class ActorControl(Node):
    def __init__(self):
        super().__init__('actor_control')
        self.position_control_mode = self.declare_parameter('position_control_mode',True).get_parameter_value().bool_value
        self.topic_actor_velocity = self.declare_parameter('topic_actor_velocity', 'cmd_vel_actor').value
        self.topic_actor_position = self.declare_parameter('topic_actor_position', 'pose_control_actor').value
        self.topic_actor_position_mode = self.declare_parameter('topic_actor_position_mode', 'mode_position_actor').value
        self.topic_actor_global_pose = self.declare_parameter('topic_actor_global_pose', 'human_global_pose').value
        self.speed = self.declare_parameter('speed',0.8).get_parameter_value().double_value
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.actor_velocity_control = self.create_publisher(Twist, self.topic_actor_velocity, 3)
        self.actor_position_control = self.create_publisher(Pose, self.topic_actor_position, 3)
        self.actor_position_mode = self.create_publisher(Bool,  self.topic_actor_position_mode, 3)
        self.mode = Bool()
        self.mode.data=self.position_control_mode

         # Subscriber đến /human_global_pose với message type Pose
        self.human_pose_subscriber = self.create_subscription(
            Pose,
            self.topic_actor_global_pose,
            self.human_pose_callback,
            5
        )

        # Waypoints - danh sách các vị trí Pose
        self.declare_parameter('waypoints', '[[0.0, 0.0, 0.0]]')  # 🔹 Thêm dòng này
        self.waypoints = self.create_waypoints()
        self.current_index = 0  # index waypoint hiện tại
        self.current_human_pose = None  # lưu pose hiện tại nhận được từ human
        


    def create_waypoints(self):
        waypoint_str = self.get_parameter('waypoints').get_parameter_value().string_value

        try:
            waypoint_array = ast.literal_eval(waypoint_str)  # safely parse string to list
        except Exception as e:
            self.get_logger().error(f"Failed to parse waypoints string: {e}")
            return []

        waypoints = []
        for coords in waypoint_array:
            if len(coords) != 3:
                self.get_logger().warn(f"Skipping invalid waypoint: {coords}")
                continue
            pose = Pose()
            pose.position = Point(x=coords[0], y=coords[1], z=coords[2])
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            waypoints.append(pose)

        self.get_logger().info(f"Loaded {len(waypoints)} waypoints from parameters.")
        return waypoints
    

    def human_pose_callback(self, msg: Pose):
        self.current_human_pose = msg
        # self.get_logger().info(f"Received human pose: position=({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f})")

    def distance(self, p1: Point, p2: Point) -> float:
        return math.sqrt(
            (p1.x - p2.x) ** 2 +
            (p1.y - p2.y) ** 2 +
            (p1.z - p2.z) ** 2
        )

    def timer_callback(self):
        if not self.position_control_mode:
            return 

        if self.current_human_pose is None:
            self.get_logger().info("Waiting for human pose...")
            return

        self.actor_position_mode.publish(self.mode)

        target_pose = self.waypoints[self.current_index]

        dist = self.distance(self.current_human_pose.position, target_pose.position)
        self.get_logger().info(f"Distance to waypoint {self.current_index}: {dist:.3f}")

        if dist < 0.05:
            if self.current_index < len(self.waypoints) - 1:
                self.current_index += 1
                self.get_logger().info(f"Switching to waypoint {self.current_index}")
            else:
                self.get_logger().info("Reached final waypoint. Stopping.")
                return  

        self.actor_position_control.publish(target_pose)
        twist = Twist()
        twist.linear.x = self.speed 
        # print(self.speed)
        self.actor_velocity_control.publish(twist)
       

        
def main(args=None):
    rclpy.init(args=args)
    node = ActorControl()
    rclpy.spin(node)

if __name__ == '__main__':
    main()