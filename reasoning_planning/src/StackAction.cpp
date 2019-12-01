#include "StackAction.h"

#include <manipulation/kmr19_put_down.h>



/* constructor */
StackAction::StackAction(ros::NodeHandle &nh) {
    // perform setup
}

/* action dispatch callback */
bool StackAction::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

    // The action implementation goes here.

    // complete the action
    ROS_INFO("ReasoningPlanning: (%s) Action callback.", msg->name.c_str());

    size_t num_values = msg->parameters.size();
    assert(num_values == 2 && "number of values for pickup must be two");

    std::string block_top_name = msg->parameters[0].value;
    std::string block_bottom_name = msg->parameters[1].value;

    Pose block_bottom_upper_face_center_pose;
    perception::Block block_top;

    if (!get_block_top_position(block_bottom_name, block_bottom_upper_face_center_pose) || !get_block(block_top_name, block_top))
    {
        return false;
    }

    ROS_INFO("stack: block_top: %s; block_bottom: %s", block_top_name.c_str(), block_bottom_name.c_str());
    ROS_INFO("bottom block upper face pose: x = %f; y = %f; z = %f", block_bottom_upper_face_center_pose.position.x, block_bottom_upper_face_center_pose.position.y, block_bottom_upper_face_center_pose.position.z);

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<manipulation::kmr19_put_down>("kmr19_manipulation_put_down");

    manipulation::kmr19_put_down srv;

    geometry_msgs::PoseStamped end_position_msg;


    end_position_msg.header.frame_id = "/world";
    end_position_msg.pose.position.x = block_bottom_upper_face_center_pose.position.x;
    end_position_msg.pose.position.y = block_bottom_upper_face_center_pose.position.y;
    end_position_msg.pose.position.z = block_bottom_upper_face_center_pose.position.z + block_top.height/2.;


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

    ros::init(argc, argv, "stack_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    StackAction rpti(nh);

    rpti.runActionInterface();

    return 0;
}