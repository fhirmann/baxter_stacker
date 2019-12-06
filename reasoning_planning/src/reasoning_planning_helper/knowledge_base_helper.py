import rospy

from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest

from rosplan_knowledge_msgs.msg import KnowledgeItem

from std_srvs.srv import Empty

from diagnostic_msgs.msg import KeyValue

# get service handle of update

service_names = {'/rosplan_knowledge_base/update'}

for service_name in service_names:
    print 'waiting for service {}'.format(service_name)
    rospy.wait_for_service(service_name)
    print 'service {} is now running'.format(service_name)

update_service = rospy.ServiceProxy(
    '/rosplan_knowledge_base/update', KnowledgeUpdateService)


def add_instance(instance_type, instance_name):
    instance_item = KnowledgeItem()
    instance_item.knowledge_type = KnowledgeItem.INSTANCE
    instance_item.instance_type = instance_type
    instance_item.instance_name = instance_name

    result = update_service(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, instance_item)

    print 'result of adding instance \'{}\' of type \'{}\''.format(instance_name, instance_type)

    print result

    return result.success



def service_update_fact_item(update_type, name, keys=[], values=[]):
    update_item = KnowledgeItem()
    update_item.knowledge_type = KnowledgeItem.FACT
    update_item.attribute_name = name

    assert(len(keys) == len(values))

    for i in range(len(keys)):
        key_val = KeyValue(key=keys[i], value=values[i])

        update_item.values.append(key_val)

    result = update_service(update_type, update_item)

    update_type_names = {KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE: 'ADD_KNOWLEDGE', 
                         KnowledgeUpdateServiceRequest.ADD_GOAL: 'ADD_GOAL', 
                         KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE: 'REMOVE_KNOWLEDGE',
                         KnowledgeUpdateServiceRequest.REMOVE_GOAL: 'REMOVE_GOAL', 
                         KnowledgeUpdateServiceRequest.ADD_METRIC: 'ADD_METRIC', 
                         KnowledgeUpdateServiceRequest.REMOVE_METRIC: 'REMOVE_METRIC'}

    print 'result of {} for \'{}'.format(update_type_names[update_type], name),

    for value in values:
        print ' {}'.format(value),

    print '\''

    print result


def update_fact_hand_empty(update_type):
    service_update_fact_item(update_type, 'hand_empty')


def update_fact_on_table(update_type, block, location):
    service_update_fact_item(update_type, 'on_table', [
                             'x', 'loc'], [block, location])


def update_fact_on(update_type, block_top, block_below):
    service_update_fact_item(update_type, 'on', ['x', 'y'], [
                             block_top, block_below])


def update_fact_clear(update_type, place):
    service_update_fact_item(update_type, 'clear', ['x'], [place])
