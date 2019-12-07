import rospy

from knowledge_base_helper import *
from rosplan_service_call_helper import *


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

service_names = {'/rosplan_knowledge_base/clear', \
                    '/rosplan_knowledge_base/update_array'}

for service_name in service_names:
    print 'waiting for service {}'.format(service_name)
    rospy.wait_for_service(service_name)
    print 'service {} is now running'.format(service_name)


def test_create_init_knowledge_base():

    # for test create knowledge programmatically as defined statically in problem_baxter.pddl

    # clear current knowledge

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

    rospy.loginfo('return of adding instances of blocks:')
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

    rospy.loginfo('result of add for \'hand_empty\'')
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

    rospy.loginfo('result of add for \'on_table yellow_block loc1\'')
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

    rospy.loginfo('return of adding instances of blocks:')
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

def test_set_goal_for_stacked_test_scene():
    # set some goal statically from stacked test scene (see perception_test_service_node, function block_list_stacked_scene)

    update_fact_hand_empty(KnowledgeUpdateServiceRequest.ADD_GOAL)

    update_fact_on_table(KnowledgeUpdateServiceRequest.ADD_GOAL, 'block1', 'loc2')
    update_fact_on_table(KnowledgeUpdateServiceRequest.ADD_GOAL, 'block2', 'loc1')
    update_fact_on(KnowledgeUpdateServiceRequest.ADD_GOAL,'block3', 'block2')