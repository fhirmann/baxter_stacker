#!/usr/bin/env python  
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_camera_link_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        br.sendTransform((0.14, 0.1, 0.835),
                         tf.transformations.quaternion_from_euler(0.0, 0.9, 0.0),
                         rospy.Time.now(),
                         "camera_link",
                         "world")

        #print("running")
        rate.sleep()
