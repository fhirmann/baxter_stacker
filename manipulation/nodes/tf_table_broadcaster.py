#!/usr/bin/env python  
import rospy
import tf
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_table_broadcaster')
   
    
    br = tf2_ros.StaticTransformBroadcaster()
    trans_stamped = geometry_msgs.msg.TransformStamped()

    trans_stamped.header.stamp = rospy.Time.now()
    trans_stamped.header.frame_id = "world"
    trans_stamped.child_frame_id = "table"

    trans_stamped.transform.translation.x = 0.485
    trans_stamped.transform.translation.y = -0.507
    trans_stamped.transform.translation.z = -0.21

    trans_stamped.transform.rotation.x = 0.0
    trans_stamped.transform.rotation.y = 0.0
    trans_stamped.transform.rotation.z = 0.0
    trans_stamped.transform.rotation.w = 1.0

    br.sendTransform(trans_stamped)
    rospy.spin()
    '''

    
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        br.sendTransform((0.485, -0.507, -0.21),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "table",
                         "world")
        rate.sleep()
    '''
    
