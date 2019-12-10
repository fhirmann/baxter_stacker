#!/usr/bin/env python
import rospy

#from reasoning_planning_helper.reasoning_planning_interface import *
from nltk.parse import CoreNLPParser
import nltk

from reasoning_planning.srv import StackGoalService, StackGoalServiceResponse, StackGoalServiceRequest

from std_msgs.msg import String

parser = CoreNLPParser(url='http://localhost:9000')

action = []
top_block = []
below_block = []
position = []

#Assign a semantic value to the request req depending on the string s
def assignServiceValue(req, s, which_block):
    
    int_value = -1
    if (s == "red" or s == "Red"):
        int_value = req.RED
    elif (s == "blue" or s == "Blue"):
        int_value = req.BLUE
    elif (s == "yellow" or s == "Yellow"):
        int_value = req.YELLOW
    elif (s == "green" or s == "Green"):
        int_value = req.GREEN
    elif (s == "left"):
        int_value = req.LEFT_RELATIVE
    elif (s == "right"):
        int_value = req.RIGHT_RELATIVE
    elif (s == "far"):
        int_value = req.FAR_RELATIVE
    elif (s == "close"):
        int_value = req.CLOSE_RELATIVE
    elif (s == "higher"):
        int_value = req.HIGHER_RELATIVE
    elif (s == "lower"):
        int_value = req.LOWER_RELATIVE
        
    if (int_value != -1):
        if (which_block == 0):
            req.top_block_semantics.append(int_value)
        elif (which_block == 1):
            req.below_block_semantics.append(int_value)
        
    return req
    
#Use a recursive function to separate the elements from the parsed tree into exploitable data (4 lists containing the action, the position and the info for each block)  
def getNodes(parent):
    ROOT = 'ROOT'
    global action, top_block, below_block, position
    for node in parent:
        if type(node) is nltk.Tree:
            if node.label() != ROOT:
                if (node.label() == 'VB'):
                    action = node.leaves()
                elif (node.label() == 'IN'):
                    position = node.leaves()
                elif (node.label() == 'NP'):
                    if (len(top_block) == 0):
                        top_block = node.leaves()
                    else : 
                        below_block = node.leaves()
            getNodes(node)

#Calls the getnode function and creates a list containing the 4 previous lists
def getRequestList(tree):
    global action, top_block, below_block, position
    getNodes(tree)    
    final_list = [action, top_block, position, below_block]
    return(final_list)

#Sends the finalized request to the service
def sendToService(req):

    rospy.loginfo("Waiting for service /stack_goal_service...")
    rospy.wait_for_service('/stack_goal_service')

    rospy.loginfo("Service /stack_goal_service is online!")
    stack_goal_service = rospy.ServiceProxy('/stack_goal_service', StackGoalService)

    req = create_simple_test_scene_request_correct()

    rospy.loginfo("request:")
    print (req)

    response = stack_goal_service(req)

    rospy.loginfo("response:")
    print (response)
 
#Callback function from the listener. Applies the Stanford Parser and creates the request
def parse_create_scene(data):
    global action, top_block, position, below_block
    action = []
    top_block = []
    below_block = []
    position = []
    
    rospy.loginfo(rospy.get_caller_id() + "I heard : %s", data.data)
    request_parsed = list(parser.parse(data.data.split()))
    request_list = getRequestList(request_parsed)
    print(request_list)
    
    req = StackGoalServiceRequest()
    for i in range (len(request_list[1])): 
        req = assignServiceValue(req, request_list[1][i], 0)
    for i in range (len(request_list[3])): 
        req = assignServiceValue(req, request_list[3][i], 1)
    print(req.top_block_semantics)
    print(req.below_block_semantics)
    
    sendToService(req)

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
