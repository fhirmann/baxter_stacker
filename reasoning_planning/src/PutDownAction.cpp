#include "PutDownAction.h"

#include <manipulation/kmr19_put_down.h>



/* constructor */
PutDownAction::PutDownAction(ros::NodeHandle &nh) {
    // perform setup
}

/* action dispatch callback */
bool PutDownAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

    // The action implementation goes here.

    // complete the action
    ROS_INFO("ReasoningPlanning: (%s) Action callback.", msg->name.c_str());

    size_t num_values = msg->parameters.size();
    assert(num_values == 2 && "number of values for pickup must be two");

    std::string block_name = msg->parameters[0].value;
    std::string location_name = msg->parameters[1].value;

    Pose location_pose;
    perception::Block block;

    if (!get_location_pose(location_name, location_pose) || !get_block(block_name, block))
    {
        return false;
    }


    ROS_INFO("put down: block: %s; location: %s", block_name.c_str(), location_name.c_str());
    ROS_INFO("location: x = %f; y = %f", location_pose.position.x, location_pose.position.y);
    ROS_INFO("block: id = %lu; height = %f", block.id, block.height);

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<manipulation::kmr19_put_down>("kmr19_manipulation_put_down");

    manipulation::kmr19_put_down srv;

    geometry_msgs::PoseStamped end_position_msg;


    end_position_msg.header.frame_id = "/world";
    end_position_msg.pose.position.x = location_pose.position.x;
    end_position_msg.pose.position.y = location_pose.position.y;
    end_position_msg.pose.position.z = s_WORLD_TO_TABLE_DISTANCE_Z + block.height/2.;


    srv.request.end_position = end_position_msg;

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

    ros::init(argc, argv, "put_down_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    PutDownAction rpti(nh);

    rpti.runActionInterface();

    return 0;
}