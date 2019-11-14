import rospy

from rosplan_dispatch_msgs.srv import DispatchService

from std_srvs.srv import Empty


rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
rospy.wait_for_service('/rosplan_planner_interface/planning_server')
rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')

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


