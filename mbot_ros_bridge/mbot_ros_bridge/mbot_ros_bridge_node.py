import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf2_ros
import math
import lcm
from mbot_lcm_msgs.mbot_imu_t import mbot_imu_t
from mbot_lcm_msgs.pose2D_t import pose2D_t 
from mbot_lcm_msgs.twist2D_t import twist2D_t

class MbotBridgeNode(Node):
    def __init__(self):
        super().__init__('mbot_ros_bridge_node')
        self.get_logger().info('Initializing Mbot Bridge Node (LCM <-> ROS2)')
        
        # Create ROS2 publishers
        self.imu_publisher = self.create_publisher(Imu, 'imu', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        
        # Create ROS2 subscribers
        self.cmd_vel_subscription = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_vel_callback, 
            10)
        
        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Initialize LCM
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
        
        # Subscribe to LCM topics
        self.lc.subscribe("MBOT_IMU", self.imu_callback)
        self.lc.subscribe("MBOT_ODOMETRY", self.odometry_callback)
        
        # Create timer for LCM handling
        self.create_timer(0.01, self.handle_lcm)  # 100Hz timer

    def handle_lcm(self):
        self.lc.handle()

    def imu_callback(self, channel, data):
        try:
            msg = mbot_imu_t.decode(data)
            
            # Create ROS2 IMU message
            imu_msg = Imu()
            
            # Set header
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Set angular velocity (gyro)
            imu_msg.angular_velocity.x = msg.gyro[0]
            imu_msg.angular_velocity.y = msg.gyro[1]
            imu_msg.angular_velocity.z = msg.gyro[2]
            
            # Set linear acceleration
            imu_msg.linear_acceleration.x = msg.accel[0]
            imu_msg.linear_acceleration.y = msg.accel[1]
            imu_msg.linear_acceleration.z = msg.accel[2]
            
            # Set orientation (using quaternion)
            imu_msg.orientation.x = msg.angles_quat[0]
            imu_msg.orientation.y = msg.angles_quat[1]
            imu_msg.orientation.z = msg.angles_quat[2]
            imu_msg.orientation.w = msg.angles_quat[3]
            
            # Publish the message
            self.imu_publisher.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {str(e)}')

    def odometry_callback(self, channel, data):
        try:
            msg = pose2D_t.decode(data)
            
            current_time = self.get_clock().now()
            
            # Create ROS2 Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_footprint"
            
            # Set position
            odom_msg.pose.pose.position.x = msg.x
            odom_msg.pose.pose.position.y = msg.y
            odom_msg.pose.pose.position.z = 0.0  # Assuming 2D robot
            
            # Set orientation (convert theta to quaternion)
            quat = self.euler_to_quaternion(0.0, 0.0, msg.theta)  # Only yaw rotation
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]
            
            # If your LCM message includes velocity, add it here
            # odom_msg.twist.twist.linear.x = msg.vx
            # odom_msg.twist.twist.linear.y = msg.vy
            # odom_msg.twist.twist.angular.z = msg.vtheta
            
            # Publish odometry message
            self.odom_publisher.publish(odom_msg)
            
            # Broadcast TF transform
            self.broadcast_tf_transform(msg.x, msg.y, msg.theta, current_time)
            
        except Exception as e:
            self.get_logger().error(f'Error processing odometry data: {str(e)}')

    def cmd_vel_callback(self, msg):
        try:
            # Create LCM twist message
            lcm_twist = twist2D_t()
            lcm_twist.utime = int(self.get_clock().now().nanoseconds / 1000)  # Convert to microseconds
            lcm_twist.vx = msg.linear.x
            lcm_twist.wz = msg.angular.z
            
            # Publish to LCM
            self.lc.publish("MBOT_VEL_CMD", lcm_twist.encode())
            
        except Exception as e:
            self.get_logger().error(f'Error sending cmd_vel to LCM: {str(e)}')

    def broadcast_tf_transform(self, x, y, theta, timestamp):
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        
        # Set translation
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        
        # Set rotation (quaternion)
        q = self.euler_to_quaternion(0.0, 0.0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0, 0, 0, 0]
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        q[3] = cr * cp * cy + sr * sp * sy  # w
        
        return q

def main(args=None):
    rclpy.init(args=args)
    node = MbotBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()