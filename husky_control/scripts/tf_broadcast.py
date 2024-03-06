#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import ModelStates

def broadcast_transforms(model_states):
    br = tf.TransformBroadcaster()
    for i, name in enumerate(model_states.name):
        if "husky_" in name:
            # Extract the position and orientation of the robot
            position = model_states.pose[i].position
            orientation = model_states.pose[i].orientation

            # Broadcast the transform from the world frame to the robot's base_link frame
            br.sendTransform((position.x, position.y, position.z),
                             (orientation.x, orientation.y, orientation.z, orientation.w),
                             rospy.Time.now(),
                             f"{name}/base_link",
                             "world")

def model_state_callback(model_states):
    broadcast_transforms(model_states)

def listener():
    rospy.init_node('husky_tf_broadcaster', anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()