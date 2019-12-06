#!/usr/bin/env python  
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_table_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.497, -0.502, -0.21),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "table",
                         "world")
        rate.sleep()