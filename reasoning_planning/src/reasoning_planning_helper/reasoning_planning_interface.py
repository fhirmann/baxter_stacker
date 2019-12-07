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

from knowledge_base_helper import *
from rosplan_service_call_helper import *

from test_functions import *


from std_msgs.msg import String
from std_srvs.srv import Empty


from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArrayRequest

from diagnostic_msgs.msg import KeyValue

from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion

from tf.transformations import *

from perception.srv import GetScene, GetSceneResponse
from perception.msg import Block

service_names = {'/get_scene', \
                    '/rosplan_knowledge_base/clear'}

for service_name in service_names:
    print 'waiting for service {}'.format(service_name)
    rospy.wait_for_service(service_name)
    print 'service {} is now running'.format(service_name)

def plan_printer(plan):
    rospy.loginfo("Printing created plan:")
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
    

def clear_blocks_from_scene_db():
    # clear stored pose informations in scene database (mongo db)
    msg_store = MessageStoreProxy()
    res = msg_store.query(Block._type)

    for (block, meta) in res:
        print meta

        block_id = meta['_id']
        msg_store.delete(str(block_id))

def clear_locations_from_scene_db():
    # clear stored location informations in scene database (mongo db)
    msg_store = MessageStoreProxy()
    res = msg_store.query(Pose._type)

    for (pose, meta) in res:
        print meta

        location_id = meta['_id']
        msg_store.delete(str(location_id))

def get_scene_and_store_in_db():
    
    get_scene_service = rospy.ServiceProxy('/get_scene', GetScene)
    result = get_scene_service()

    print result
    if result.success:

        # clear previous items in scene db and knowledge base
        clear_blocks_from_scene_db()
        clear_locations_from_scene_db()

        clear_service = rospy.ServiceProxy('/rosplan_knowledge_base/clear', Empty)
        result_clear = clear_service()

        print "result_clear = "
        print result_clear

        msg_store = MessageStoreProxy()

        for block in result.blocks:

            msg_store.insert_named('block{}'.format(block.id), block)
    

    return result.success   

def is_epsilon_close(val1, val2, epsilon):
    return abs(val1 - val2) < epsilon

def is_close_location(position1, position2, epsilon):
    return is_epsilon_close(position1.x, position2.x, epsilon) and is_epsilon_close(position1.y, position2.y, epsilon)

def create_knowledge_from_scene_db():

    msg_store = MessageStoreProxy()
    res = msg_store.query(Block._type)

    # sort them by z direction from lowest to most upper to have always lowest created before the upper ones
    sorted_results = res.sort(key=lambda x: x[0].pose.pose.position.z)

    for (block, meta) in res:

        block_instance_name = "block{}".format(block.id)

        if not add_instance("block", block_instance_name):
            return False
        
        # check if on table

        pos = block.pose.pose.position

        lower_face_pose_z = pos.z - block.height / 2.

        table_height = -0.2

        epsilon = 10./1000

        if is_epsilon_close(lower_face_pose_z, table_height, epsilon):
            # add new location position
            location_instance_name = "loc{}".format(block.id)
            add_instance("location", location_instance_name)#

            add_location(location_instance_name, pos.x, pos.y)

            update_fact_on_table(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, block_instance_name, location_instance_name)

            # check if there's a block on top of it
            upper_face_pose_z = pos.z + block.height / 2.
            is_block_on_top = False
            for (other_block, other_meta) in res:

                if meta['_id'] == other_meta['_id']:
                    continue
                

                other_block_position = other_block.pose.pose.position

                if is_close_location(pos, other_block_position, epsilon) and is_epsilon_close(upper_face_pose_z, other_block_position.z - other_block.height/2, epsilon):
                    is_block_on_top = True
                    break

            if not is_block_on_top:
                update_fact_clear(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, block_instance_name)

        else: # it's not directly on the table

            # check on which block it is and if there's a block on top of it
            upper_face_pose_z = pos.z + block.height / 2.
            lower_face_pose_z = pos.z - block.height / 2.
            is_block_on_top = False

            for (other_block, other_meta) in res:

                if meta['_id'] == other_meta['_id']:
                    continue
                

                other_block_position = other_block.pose.pose.position

                if is_close_location(pos, other_block_position, epsilon):
                    
                    if is_epsilon_close(upper_face_pose_z, other_block_position.z - other_block.height/2, epsilon):
                        is_block_on_top = True
                    elif is_epsilon_close(lower_face_pose_z, other_block_position.z + other_block.height/2, epsilon):
                        update_fact_on(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, block_instance_name, "block{}".format(other_block.id))
            

            if not is_block_on_top:
                update_fact_clear(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, block_instance_name)


def print_blocks_from_db():
    msg_store = MessageStoreProxy()
    res = msg_store.query(Block._type)

    rospy.loginfo("stored blocks in scene DB are:")
    for (block, meta) in res:
        print meta
        print block

def get_all_blocks_from_db():
    msg_store = MessageStoreProxy()
    res = msg_store.query(Block._type)

    (blocks, metas) = zip(*res)


    return blocks
        

def add_init_knowledge():
    update_fact_hand_empty(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
    # add one extra location for temporary moving to that position
    add_location('loc_temp', 0.6, 0.2) # TODO: set some good position for this temporary location
    update_fact_clear(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, 'loc_temp')

