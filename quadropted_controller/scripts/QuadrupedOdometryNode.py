#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import Float64, Int64
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
import tf_transformations
import tf2_ros
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from quadropted_msgs.msg import RobotVelocity

class DogOdometry(Node):
    def __init__(self):
        super().__init__('dog_odometry')
        try:
            self.declare_parameter('verbose', False)
            self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
            if self.verbose:
                self.get_logger().info(f"Verbose mode: {self.verbose}")

            self.declare_parameter('publish_rate', 50)
            publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value
            if self.verbose:
                self.get_logger().info(f"Publish rate: {publish_rate} Hz")

            self.declare_parameter('has_imu_heading', True)
            self.has_imu_heading = self.get_parameter('has_imu_heading').get_parameter_value().bool_value
            if self.verbose:
                self.get_logger().info(f"Has IMU heading: {self.has_imu_heading}")

            self.declare_parameter('enable_odom_tf', True)
            self.enable_odom_tf = self.get_parameter('enable_odom_tf').get_parameter_value().bool_value
            if self.verbose:
                self.get_logger().info(f"Enable odom TF: {self.enable_odom_tf}")

            self.declare_parameter('base_frame_id', 'base')
            self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
            if self.verbose:
                self.get_logger().info(f"Base frame ID: {self.base_frame_id}")

            self.declare_parameter('odom_frame_id', 'odom')
            self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
            if self.verbose:
                self.get_logger().info(f"Odom frame ID: {self.odom_frame_id}")

            self.declare_parameter('is_gazebo', True)
            self.is_gazebo = self.get_parameter('is_gazebo').get_parameter_value().bool_value
            if self.verbose:
                self.get_logger().info(f"Is Gazebo: {self.is_gazebo}")

            self.declare_parameter('clock_topic', '/clock')
            clock_topic = self.get_parameter('clock_topic').get_parameter_value().string_value
            if self.verbose:
                self.get_logger().info(f"Clock Topic: {clock_topic}")

            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.linear_velocity_x = 0.0
            self.linear_velocity_y = 0.0
            self.angular_velocity = 0.0

            self.last_position_time = self.get_clock().now()

            self.gazebo_clock = Time()
            self.encoder_pos = 0

            qos_reliable = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=10,
                history=HistoryPolicy.KEEP_LAST
            )
            qos_best_effort = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10,
                history=HistoryPolicy.KEEP_LAST
            )

            self.odom_pub = self.create_publisher(Odometry, 'odom', qos_reliable)

            if self.has_imu_heading:
                self.imu_sub = self.create_subscription(
                    Imu,
                    'imu_plugin/out',
                    self.imu_callback,
                    qos_reliable
                )

            self.velocity_sub = self.create_subscription(
                RobotVelocity,
                'robot_velocity',
                self.velocity_callback,
                qos_reliable
            )

            if self.enable_odom_tf:
                self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

            if self.is_gazebo:
                clock_qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10,
                    history=HistoryPolicy.KEEP_LAST
                )
                self.clock_sub = self.create_subscription(
                    Clock,
                    clock_topic,
                    self.clock_callback,
                    clock_qos
                )
                if self.verbose:
                    self.get_logger().info("Subscribed to /clock topic with BEST_EFFORT QoS.")
            else:
                encoder_qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10,
                    history=HistoryPolicy.KEEP_LAST
                )
                self.encoder_sub = self.create_subscription(
                    Int64,
                    'encoder_value',
                    self.encoder_callback,
                    encoder_qos
                )
                if self.verbose:
                    self.get_logger().info("Subscribed to encoder_value topic with BEST_EFFORT QoS.")

            timer_period = 1.0 / publish_rate
            self.timer = self.create_timer(timer_period, self.timer_callback)

            self.get_logger().info("Dog Odometry Node has been started.")

            self.MAX_LINEAR_VELOCITY_X = 1.0
            self.MAX_LINEAR_VELOCITY_Y = 1.0
            self.MAX_ANGULAR_VELOCITY = 1.0

            self.imu_angular_velocity = 0.0

        except Exception as e:
            self.get_logger().error(f"Exception during node initialization: {e}")
            raise e

    def velocity_callback(self, msg):
        self.linear_velocity_x = (msg.cmd_vel.linear.x / 0.035) / 3
        self.linear_velocity_y = (msg.cmd_vel.linear.y / 0.012) / 3
        if self.verbose:
            self.get_logger().info(f"Robot Velocity - Linear X: {self.linear_velocity_x:.6f} m/s, Linear Y: {self.linear_velocity_y:.6f} m/s")

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)

        self.theta = yaw
        self.imu_angular_velocity = -msg.angular_velocity.z

        if self.verbose:
            self.get_logger().info(f"IMU Yaw: {self.theta:.6f} rad")
            self.get_logger().info(f"IMU Angular Velocity: {self.imu_angular_velocity:.6f} rad/s")

    def clock_callback(self, msg):
        self.gazebo_clock = msg.clock
        if self.verbose:
            self.get_logger().info(f"Received Gazebo Clock: {self.gazebo_clock.sec}.{self.gazebo_clock.nanosec}")

    def encoder_callback(self, msg):
        self.encoder_pos = msg.data
        if self.verbose:
            self.get_logger().info(f"Received Encoder Position: {self.encoder_pos}")

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_position_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        self.linear_velocity_x = max(min(self.linear_velocity_x, self.MAX_LINEAR_VELOCITY_X), -self.MAX_LINEAR_VELOCITY_X)
        self.linear_velocity_y = max(min(self.linear_velocity_y, self.MAX_LINEAR_VELOCITY_Y), -self.MAX_LINEAR_VELOCITY_Y)
        self.imu_angular_velocity = max(min(self.imu_angular_velocity, self.MAX_ANGULAR_VELOCITY), -self.MAX_ANGULAR_VELOCITY)

        delta_x = (self.linear_velocity_x * math.cos(self.theta) - self.linear_velocity_y * math.sin(self.theta)) * dt
        delta_y = (self.linear_velocity_x * math.sin(self.theta) + self.linear_velocity_y * math.cos(self.theta)) * dt
        self.x += delta_x
        self.y += delta_y

        odom = Odometry()
        if self.is_gazebo:
            odom.header.stamp = self.gazebo_clock
        else:
            odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )

        odom.twist.twist.linear.x = self.linear_velocity_x
        odom.twist.twist.linear.y = self.linear_velocity_y
        odom.twist.twist.angular.z = self.imu_angular_velocity

        self.odom_pub.publish(odom)

        if self.enable_odom_tf:
            t = TransformStamped()
            if self.is_gazebo:
                t.header.stamp = self.gazebo_clock
            else:
                t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = Quaternion(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3]
            )

            self.tf_broadcaster.sendTransform(t)

        self.last_position_time = current_time

        if self.verbose:
            self.get_logger().info(f"Position Updated: x={self.x:.6f} m, y={self.y:.6f} m, theta={self.theta:.6f} rad")
            self.get_logger().info(f"Delta_x: {delta_x:.6f} m, Delta_y: {delta_y:.6f} m")
            self.get_logger().info(f"Robot Velocity - Linear X: {self.linear_velocity_x:.6f} m/s, Linear Y: {self.linear_velocity_y:.6f} m/s")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = DogOdometry()
    except Exception as e:
        print(f"Exception during node initialization: {e}")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
