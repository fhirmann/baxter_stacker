#include "PickUpAction.h"

#include <random>

#include <diagnostic_msgs/KeyValue.h>
#include <string>


/* constructor */
PickUpAction::PickUpAction(ros::NodeHandle &nh) {
    // perform setup
}

/* action dispatch callback */
bool PickUpAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

    // The action implementation goes here.

    // complete the action
    ROS_INFO("ReasoningPlanning: (%s) Action callback.", msg->name.c_str());

    size_t num_values = msg->parameters.size();
    assert(num_values == 2 && "number of values for pickup must be two");

    std::string block = msg->parameters[0].value;
    std::string location = msg->parameters[1].value;

    Pose p;

    if (!get_location_pose(location, p))
    {
        return false;
    }


    ROS_INFO("pick up: block: %s; location: %s", block.c_str(), location.c_str());
    ROS_INFO("x = %f; y = %f", p.position.x, p.position.y);

    return ((double)rand() / RAND_MAX) > 0.9; // success with probability of 80 %
}


/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "pick_up_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    PickUpAction rpti(nh);

    rpti.runActionInterface();

    return 0;
}