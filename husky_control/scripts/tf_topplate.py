#!/usr/bin/env python
import rospy
import tf
import tf.transformations as tr

class TopPlateLinkFixer:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.rate = rospy.Rate(30.0)  # 30 Hz
        self.N = 3

    def run(self):
        while not rospy.is_shutdown():
            for i in range(self.N):  # Assuming N = 0, 1, 2
                try:
                    # Listen for the original transform
                    (trans, rot) = self.listener.lookupTransform(f'/husky_{i}/base_link', f'/husky_{i}/top_plate_link', rospy.Time(0))
                    
                    # Convert quaternion to Euler
                    euler = tr.euler_from_quaternion(rot)
                    
                    # Reset roll and pitch to zero while keeping yaw unchanged
                    new_quat = tr.quaternion_from_euler(0.0, 0.0, euler[2])

                    # Broadcast the new transform
                    self.broadcaster.sendTransform(trans,
                                                   new_quat,
                                                   rospy.Time.now(),
                                                   f'/husky_{i}/top_plate_link_fix',
                                                   f'/husky_{i}/base_link')
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                
                self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('top_plate_link_fixer', anonymous=True)
    fixer = TopPlateLinkFixer()
    fixer.run()