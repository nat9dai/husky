#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def set_model_state(model_name, position, orientation):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose.position.x = position['x']
        model_state.pose.position.y = position['y']
        model_state.pose.position.z = position['z']
        model_state.pose.orientation.x = orientation['x']
        model_state.pose.orientation.y = orientation['y']
        model_state.pose.orientation.z = orientation['z']
        model_state.pose.orientation.w = orientation['w']

        resp = set_state(model_state)
        rospy.loginfo(f"Model {model_name} state updated: {resp.success}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('reset_pose', anonymous=True)
    
    models = [
        {
            'name': 'husky_0',
            'position': {'x': -5.745344638275217, 'y': -5.128108664755878, 'z': 0.18227963606616213},
            'orientation': {'x': -2.072996208794327e-06, 'y': 1.0350293693687206e-06, 'z': 0.026077412674978748, 'w': 0.9996599264463942}
        },
        {
            'name': 'husky_1',
            'position': {'x': -6.841959771898666, 'y': -6.135236792733406, 'z': 0.18228091951027883},
            'orientation': {'x': -4.216454890422215e-06, 'y': -3.497942453577626e-07, 'z': 0.6791961938089814, 'w': 0.7339567632343827}
        },
        {
            'name': 'husky_2',
            'position': {'x': -7.704873747285005, 'y': -7.351390454079278, 'z': 0.18233583953009389},
            'orientation': {'x': -5.8911815932129086e-06, 'y': 0.000123353667787622, 'z': 0.004607578662075464, 'w': 0.9999893774276003}
        }
        # Add more objects as needed
    ]

    try:
        for model in models:
            set_model_state(model['name'], model['position'], model['orientation'])
    except rospy.ROSInterruptException:
        pass
