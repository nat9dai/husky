#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import UInt8
from gazebo_msgs.msg import ModelStates
import tf.transformations
import numpy as np
import sys

class TrajectoryPublisher:
    def __init__(self, robot_id):
        self.robot_id = int(robot_id)
        self.traj = Path()
        self.T = 10
        self.N = 3
        self.sampling_time = 0.1
        self.x, self.y, self.z, self.yaw = 0.0, 0.0, 0.0, 0.0
        self.linear_vel, self.angular_vel = 0.0, 0.0
        self.rate = rospy.Rate(30)

        self.first_yaw_reading = True
        self.last_yaw = 0.0
        self.accumulated_yaw = 0.0

        self.pi_i = UInt8()
        self.pi_i.data = 0

        self.traj_pub = rospy.Publisher(f'/husky_{robot_id}/traj', Path, queue_size=10)
        self.hierarchy_pub = rospy.Publisher(f'/husky_{robot_id}/hierarchy', UInt8, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.mocap_callback)
        rospy.Subscriber(f'/husky_{robot_id}/husky_velocity_controller/cmd_vel', Twist, self.cmd_vel_callback)

    def mocap_callback(self, msg):
        #rospy.loginfo(f"mocap_callback: lenght_pose={len(msg.pose)}, lenght_twist={len(msg.twist)}")
        # lenght = 39 (with 3 robot)
        for i, name in enumerate(msg.name):
            if "husky_" in name:
                index = int(name[6:])  # Extracts the number after "husky_"
                if index == self.robot_id:
                    quaternion = (
                        msg.pose[i].orientation.x,
                        msg.pose[i].orientation.y,
                        msg.pose[i].orientation.z,
                        msg.pose[i].orientation.w
                    )
                    euler = tf.transformations.euler_from_quaternion(quaternion)
                    roll, pitch, yaw = euler
                    if self.first_yaw_reading:
                        self.last_yaw = yaw
                        self.accumulated_yaw = yaw
                        self.first_yaw_reading = False
                    else:
                        yaw_diff = yaw - self.last_yaw
                        if yaw_diff > np.pi:
                            yaw_diff -= 2 * np.pi
                        elif yaw_diff < -np.pi:
                            yaw_diff += 2 * np.pi
                        self.accumulated_yaw += yaw_diff
                        self.last_yaw = yaw

                    self.x = msg.pose[i].position.x
                    self.y = msg.pose[i].position.y
                    self.yaw = self.accumulated_yaw

    def cmd_vel_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def dynamic(self, u):
        x_next = self.x + self.sampling_time * u[0] * np.cos(self.yaw)
        y_next = self.y + self.sampling_time * u[0] * np.sin(self.yaw)
        yaw_next = self.yaw + self.sampling_time * u[1]
        return [x_next, y_next, yaw_next]

    def publisher_loop(self):
        while not rospy.is_shutdown():
            self.traj.poses = []  # Clear the trajectory
            x_k = self.x
            y_k = self.y
            yaw_k = self.yaw
            for i in range(self.T):
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "world"  # Same frame ID as the Path
                pose.pose.position.x = x_k
                pose.pose.position.y = y_k
                # In this project, orientation x y are v_x, v_y
                pose.pose.orientation.x = self.linear_vel * np.cos(yaw_k)
                pose.pose.orientation.y = self.angular_vel* np.sin(yaw_k)

                self.traj.poses.append(pose)
                [x_k, y_k, yaw_k] = self.dynamic([self.linear_vel, self.angular_vel])
            self.traj_pub.publish(self.traj)
            self.hierarchy_pub.publish(self.pi_i)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fake_trajectory')
    robot_id = sys.argv[1]
    trajectory_publisher = TrajectoryPublisher(robot_id)
    try:
        trajectory_publisher.publisher_loop()
    except rospy.ROSInterruptException:
        pass
