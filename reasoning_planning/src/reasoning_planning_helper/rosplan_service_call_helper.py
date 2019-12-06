import rospy

from rosplan_dispatch_msgs.srv import DispatchService

from std_srvs.srv import Empty


service_names = {'/rosplan_problem_interface/problem_generation_server', \
                    '/rosplan_planner_interface/planning_server', \
                    '/rosplan_parsing_interface/parse_plan', \
                    '/rosplan_plan_dispatcher/dispatch_plan'}

for service_name in service_names:
    print 'waiting for service {}'.format(service_name)
    rospy.wait_for_service(service_name)
    print 'service {} is now running'.format(service_name)

def generate_problem():
    problem_service = rospy.ServiceProxy(
        '/rosplan_problem_interface/problem_generation_server', Empty)
    result = problem_service()

    print result

    print "Generation of problem successfully!"


def generate_plan():
    planner_service = rospy.ServiceProxy(
        '/rosplan_planner_interface/planning_server', Empty)
    result = planner_service()

    print result

    print "Generation of plan successfully!"

def parse_plan():
    parsing_service = rospy.ServiceProxy(
        '/rosplan_parsing_interface/parse_plan', Empty)
    result = parsing_service()

    print result

    print "Parsing of plan successfully!"

def dispatch_plan():
    dispatch_service = rospy.ServiceProxy(
        '/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
    result = dispatch_service()

    print result

    print "Dispatching of plan successfully!"

    return result.goal_achieved


