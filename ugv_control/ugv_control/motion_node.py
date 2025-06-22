#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import UInt8, Float32
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from ugv_control.rosmaster import Rosmaster
from ugv_control.motion_controller import MotionController
import math
from scipy.spatial.transform import Rotation

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')
        self.rosmaster = Rosmaster(com='/dev/myserial', debug=False)
        self.motion = MotionController(self.rosmaster)

        self.rosmaster.create_receive_threading()
        self.rosmaster.set_auto_report_state(True)

        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.subscription_mode = self.create_subscription(
            UInt8,
            '/motion_mode',
            self.mode_callback,
            10
        )
        self.voltage_pub = self.create_publisher(Float32, '/battery_voltage', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/magnetometer', 10)
        self.velocity_pub = self.create_publisher(Vector3Stamped, '/velocity', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu_timer = self.create_timer(0.02, self.publish_imu_data)  # 50Hz
        self.status_timer = self.create_timer(1, self.publish_robot_status)  # 1Hz
        self.odom_timer = self.create_timer(0.02, self.publish_odometry)  # 50Hz

        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.modes = ['manual', 'explore', 'goto']
        self.current_mode_index = 0
        self.last_button_state = 0

        self.get_logger().info("MotionNode started. Initial mode: manual")

    def mode_callback(self, msg: UInt8):
        index = int(msg.data)
        if index < len(self.modes) and index != self.current_mode_index:
            self.current_mode_index = index
            self.get_logger().info(f"Switched to mode via RC: {self.modes[index]}")

    def cmd_vel_callback(self, msg: Twist):
        if self.modes[self.current_mode_index] != 'manual':
            return
        self.motion.set_velocity(msg.linear.x, msg.linear.y, msg.angular.z)

    def publish_imu_data(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        ax, ay, az = self.rosmaster.get_accelerometer_data()
        gx, gy, gz = self.rosmaster.get_gyroscope_data()

        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)

        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        self.imu_pub.publish(msg)

    def publish_robot_status(self):
        now = self.get_clock().now().to_msg()

        voltage = self.rosmaster.get_battery_voltage()
        self.voltage_pub.publish(Float32(data=voltage))

        mx, my, mz = self.rosmaster.get_magnetometer_data()
        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = "imu_link"
        mag_msg.magnetic_field.x = float(mx)
        mag_msg.magnetic_field.y = float(my)
        mag_msg.magnetic_field.z = float(mz)
        self.mag_pub.publish(mag_msg)

        vx, vy, omega = self.rosmaster.get_motion_data()
        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = now
        vel_msg.header.frame_id = "base_link"
        vel_msg.vector.x = float(vx)
        vel_msg.vector.y = float(vy)
        vel_msg.vector.z = float(omega)
        self.velocity_pub.publish(vel_msg)

        self.vx = float(vx)
        self.vy = float(vy)
        self.vth = float(omega)

    def publish_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = Rotation.from_euler('z', self.th).as_quat()

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)

        t1 = TransformStamped()
        t1.header.stamp = now.to_msg()
        t1.header.frame_id = "odom"
        t1.child_frame_id = "base_footprint"
        t1.transform.translation.x = self.x
        t1.transform.translation.y = self.y
        t1.transform.translation.z = 0.0
        t1.transform.rotation.x = odom_quat[0]
        t1.transform.rotation.y = odom_quat[1]
        t1.transform.rotation.z = odom_quat[2]
        t1.transform.rotation.w = odom_quat[3]
        self.tf_broadcaster.sendTransform(t1)

        t2 = TransformStamped()
        t2.header.stamp = now.to_msg()
        t2.header.frame_id = "base_footprint"
        t2.child_frame_id = "base_link"
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.082
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t2)

def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.motion.stop()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
