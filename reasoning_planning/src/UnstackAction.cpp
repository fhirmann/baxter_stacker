#include "UnstackAction.h"

#include <random>


/* constructor */
UnstackAction::UnstackAction(ros::NodeHandle &nh) {
    // perform setup
}

/* action dispatch callback */
bool UnstackAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

    // The action implementation goes here.

    // complete the action
    ROS_INFO("ReasoningPlanning: (%s) Action callback.", msg->name.c_str());
    return ((double)rand() / RAND_MAX) > 0.8; // success with probability of 80 %
}


/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "unstack_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    UnstackAction rpti(nh);

    rpti.runActionInterface();

    return 0;
}