#!/usr/bin/env python

# START - for debugging in visual studio code
# import ptvsd

# # 5678 is the default attach port in the VS Code debug configurations
# print("Waiting for debugger attach")
# ptvsd.enable_attach(address=('localhost', 5678), redirect_output=True)
# ptvsd.wait_for_attach()
# breakpoint()

# END - for debugging in visual studio code

import rospy

from reasoning_planning_helper.knowledge_base_helper import *
from reasoning_planning_helper.rosplan_service_call_helper import *


from std_msgs.msg import String
from std_srvs.srv import Empty


from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArrayRequest

from diagnostic_msgs.msg import KeyValue

from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion

from tf.transformations import *

def plan_printer(plan):
    print "Printing plan:"
    print plan.data

def add_location(name, x, y, yaw = 0):
    if not add_instance("location", name):
        return -1

    msg_store = MessageStoreProxy()

    q = quaternion_from_euler(0,0,yaw)

    p = Pose(Point(x,y,0), Quaternion(*q))

    location_id = msg_store.insert_named(name, p)

    return location_id

def get_location_pose(name):
    msg_store = MessageStoreProxy()

    p = msg_store.query_named(name, Pose._type)

    if p == None:
        raise LookupError("Location is not found in scene database", name)

    return p
    

def test_create_init_knowledge_base():

    # for test create knowledge programmatically as defined statically in problem_baxter.pddl

    # clear current knowledge

    rospy.wait_for_service('/rosplan_knowledge_base/clear')
    clear_service = rospy.ServiceProxy('/rosplan_knowledge_base/clear', Empty)
    result = clear_service()

    # clear also stored pose informations in scene database (mongo db)
    msg_store = MessageStoreProxy()
    res = msg_store.query(Pose._type)

    for (pose, meta) in res:
        print meta

        loc_id = meta['_id']
        msg_store.delete(str(loc_id))


    # get service handle for update_array

    rospy.wait_for_service('/rosplan_knowledge_base/update_array')
    update_array_service = rospy.ServiceProxy(
        '/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)

    # add instances of blocks with array update

    instance_names = {'green_block', 'yellow_block', 'blue_block'}

    update_items = []
    update_types = []

    for instance_name in instance_names:
        update_item = KnowledgeItem()
        update_item.knowledge_type = KnowledgeItem.INSTANCE
        update_item.instance_type = 'block'
        update_item.instance_name = instance_name

        update_items.append(update_item)

        update_types.append(KnowledgeUpdateServiceArrayRequest.ADD_KNOWLEDGE)

    result = update_array_service(update_types, update_items)

    print 'return of adding instances of blocks:'
    print result

    add_location("loc1", 1, 11)
    add_location("loc2", 2, 22)
    add_location("loc3", 3, 33)

    # create init state (this time with single service call)

    update_item = KnowledgeItem()
    update_item.knowledge_type = KnowledgeItem.FACT
    update_item.attribute_name = 'hand_empty'

    result = update_service(
        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, update_item)

    print 'result of add for \'hand_empty\''
    print result

    update_item = KnowledgeItem()
    update_item.knowledge_type = KnowledgeItem.FACT
    update_item.attribute_name = 'on_table'

    val1 = KeyValue(key='x', value='yellow_block')

    val2 = KeyValue()
    val2.key = 'loc'
    val2.value = 'loc1'

    update_item.values = [val1, val2]

    result = update_service(
        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, update_item)

    print 'result of add for \'on_table yellow_block loc1\''
    print result

    # do the rest via nice helper functions

    update_fact_on(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE,
                   'blue_block', 'yellow_block')
    update_fact_clear(
        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, 'blue_block')
    update_fact_on_table(
        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, 'green_block', 'loc2')
    update_fact_clear(
        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, 'green_block')
    update_fact_clear(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, 'loc3')


def test_set_goal():
    # set goal programmatically as defined statically in problem_baxter.pddl

    update_fact_hand_empty(KnowledgeUpdateServiceRequest.ADD_GOAL)
    update_fact_on_table(
        KnowledgeUpdateServiceRequest.ADD_GOAL, 'green_block', 'loc1')
    update_fact_on(KnowledgeUpdateServiceRequest.ADD_GOAL,
                   'yellow_block', 'green_block')
    update_fact_clear(KnowledgeUpdateServiceRequest.ADD_GOAL, 'yellow_block')

def test_create_init_knowledge_base_manipulation_simple():

    # for test create knowledge programmatically as defined statically for manipulation
    # one yellow big block in the center at the marked position

    # clear current knowledge

    rospy.wait_for_service('/rosplan_knowledge_base/clear')
    clear_service = rospy.ServiceProxy('/rosplan_knowledge_base/clear', Empty)
    result = clear_service()

    # clear also stored pose informations in scene database (mongo db)
    msg_store = MessageStoreProxy()
    res = msg_store.query(Pose._type)

    for (pose, meta) in res:
        print meta

        loc_id = meta['_id']
        msg_store.delete(str(loc_id))


    # get service handle for update_array

    rospy.wait_for_service('/rosplan_knowledge_base/update_array')
    update_array_service = rospy.ServiceProxy(
        '/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)

    # add instances of blocks with array update

    instance_names = {'yellow_block'}

    update_items = []
    update_types = []

    for instance_name in instance_names:
        update_item = KnowledgeItem()
        update_item.knowledge_type = KnowledgeItem.INSTANCE
        update_item.instance_type = 'block'
        update_item.instance_name = instance_name

        update_items.append(update_item)

        update_types.append(KnowledgeUpdateServiceArrayRequest.ADD_KNOWLEDGE)

    result = update_array_service(update_types, update_items)

    print 'return of adding instances of blocks:'
    print result

    add_location("loc1", 0.93, 0.3)
    add_location("loc2", 0.81, 0.2)

    update_fact_hand_empty(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
    update_fact_on_table(
        KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, 'yellow_block', 'loc1')
    update_fact_clear(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, 'yellow_block')
    update_fact_clear(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, 'loc2')

def test_set_goal_manipulation_simple():
    update_fact_on_table(KnowledgeUpdateServiceRequest.ADD_GOAL, 'yellow_block', 'loc2')

def test_scene_and_goal_stacked_blocks():
    test_create_init_knowledge_base()

    test_set_goal()

def test_scene_and_goal_simple_manipulation():
    test_create_init_knowledge_base_manipulation_simple()

    test_set_goal_manipulation_simple()

def main():

    # init

    rospy.init_node('reasoning_planning_interface', anonymous=True)

    rospy.Subscriber(
        '/rosplan_planner_interface/planner_output', String, plan_printer)

   
    test_scene_and_goal_simple_manipulation()



    goal_reached = False
    while not goal_reached:
        generate_problem()
        generate_plan()

        parse_plan()
        goal_reached = dispatch_plan()

        goal_reached = True # hack to only do once

    print "end of main reached!"

    #rospy.spin()


if __name__ == '__main__':
    main()    msg_store = MessageStoreProxy()
