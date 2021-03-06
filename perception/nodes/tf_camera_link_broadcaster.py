#!/usr/bin/env python  
import rospy
import tf
from math import pi

if __name__ == '__main__':
    rospy.init_node('tf_camera_link_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(2.0)
    first = 0

    while not rospy.is_shutdown():
        br.sendTransform((0.165, 0.088, 0.862),
                         tf.transformations.quaternion_from_euler(0.01, 0.8825, 0.065),
                         rospy.Time.now(),
                         "camera_link",
                         "world")

        if first == 0:
            rospy.loginfo("TF_CAMERA_WORLD: transformation broadcast is running" )
            first = 1

        #print("running")
        rate.sleep()
