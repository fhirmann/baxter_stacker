#include "UnstackAction.h"

#include <manipulation/kmr19_pick_up.h>


/* constructor */
UnstackAction::UnstackAction(ros::NodeHandle &nh) {
    // perform setup
}

/* action dispatch callback */
bool UnstackAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

    // The action implementation goes here.

    // complete the action
    ROS_INFO("ReasoningPlanning: (%s) Action callback.", msg->name.c_str());

    size_t num_values = msg->parameters.size();
    assert(num_values == 2 && "number of values for unstack must be two");

    std::string block_top_name = msg->parameters[0].value;
    std::string block_below_name = msg->parameters[1].value;

    perception::Block block_top;

    if (!get_block(block_top_name, block_top))
    {
        return false;
    }


    ROS_INFO("unstack: block_top: %s; block_below: %s", block_top_name.c_str(), block_below_name.c_str());
    ROS_INFO("block_top: id = %lu", block_top.id);

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<manipulation::kmr19_pick_up>("kmr19_manipulation_pick_up");

    manipulation::kmr19_pick_up srv;
    srv.request.block_id = block_top.id;

    ROS_INFO_STREAM("request: " << srv.request);

    return true; // temporary for testing

    if (client.call(srv))
    {
        ROS_INFO("Success: %d", (int)srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service kmr19_pick_up");
        return false;
    }

    return srv.response.success;
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