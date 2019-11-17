#include "StackAction.h"

#include <random>


/* constructor */
StackAction::StackAction(ros::NodeHandle &nh) {
    // perform setup
}

/* action dispatch callback */
bool StackAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

    // The action implementation goes here.

    // complete the action
    ROS_INFO("ReasoningPlanning: (%s) Action callback.", msg->name.c_str());
    return ((double)rand() / RAND_MAX) > 0.8; // success with probability of 80 %
}


/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "stack_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    StackAction rpti(nh);

    rpti.runActionInterface();

    return 0;
}