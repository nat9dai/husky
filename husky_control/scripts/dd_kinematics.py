#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
import numpy as np

class DifferentialDriveKinemetics:
    def __init__(self):
        rospy.init_node('dd_kinemetics')

        self.N = 3
        self.x = np.zeros(self.N)
        self.y = np.zeros(self.N)
        self.theta = np.zeros(self.N)

        self.x[1] = 1.0
        self.y[1] = 3.0
        self.theta[1] = np.radians(30.0)
        self.x[2] = -3.0
        self.y[2] = -1.0
        self.theta[2] = np.radians(45.0)

        self.limit_v = 2.0
        self.limit_w = 12.0

        self.last_time = []
        for i in range(self.N):
            self.last_time.append(rospy.Time.now())

        for i in range(self.N):
            rospy.Subscriber('/husky_{}/set_states/cmd_vel'.format(i), Twist, self.cmdCallback, i)

        # Service client for the /gazebo/set_model_state service
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.rate = rospy.Rate(20)

    def cmdCallback(self, msg, robot_index):
        #current_time = rospy.Time.now()
        #delta_time = (current_time - self.last_time[robot_index]).to_sec()
        delta_time = 0.01
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Limiting linear velocity
        if linear_vel > self.limit_v:
            linear_vel = self.limit_v
        elif linear_vel < -self.limit_v:
            linear_vel = -self.limit_v

        # Limiting angular velocity
        if angular_vel > self.limit_w:
            angular_vel = self.limit_w
        elif angular_vel < -self.limit_w:
            angular_vel = -self.limit_w

        delta_x = linear_vel * np.cos(self.theta[robot_index]) * delta_time
        delta_y = linear_vel * np.sin(self.theta[robot_index]) * delta_time
        delta_theta = angular_vel * delta_time

        self.x[robot_index] += delta_x
        self.y[robot_index] += delta_y
        self.theta[robot_index] += delta_theta

        #self.last_time[robot_index] = current_time

    def setStates(self):
        for i in range(self.N):
            quaternion = quaternion_from_euler(0, 0, self.theta[i])

            # Update the robot state in Gazebo
            model_state = ModelState()
            model_state.model_name = 'husky_{}'.format(i)
            model_state.pose.position.x = self.x[i]
            model_state.pose.position.y = self.y[i]
            model_state.pose.position.z = 0.18  # Assuming 2D motion

            model_state.pose.orientation.x = quaternion[0]
            model_state.pose.orientation.y = quaternion[1]
            model_state.pose.orientation.z = quaternion[2]
            model_state.pose.orientation.w = quaternion[3]

            # Call the service
            try:
                self.set_state_service(model_state)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        node = DifferentialDriveKinemetics()
        while not rospy.is_shutdown():
            node.setStates()
            node.rate.sleep()
    except rospy.ROSInterruptException:
        pass