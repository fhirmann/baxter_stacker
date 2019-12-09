#!/usr/bin/env python
import rospy

#from reasoning_planning_helper.reasoning_planning_interface import *

from reasoning_planning.srv import StackGoalService, StackGoalServiceResponse, StackGoalServiceRequest

from std_msgs.msg import String


def parse_create_scene(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard : %s", data.data)

    #TODO
    #PARSE WITH STANFORD PARSER
    #Result will be 2 structs containing infos such as location, color
    #we will put those structs in a fction that will return the request with correct semantics
    #Then send it to the server as in the example

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("speech_request", String, parse_create_scene)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
