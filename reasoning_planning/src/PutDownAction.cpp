#include "PutDownAction.h"

#include <manipulation/kmr19_put_down.h>

#include <tf/transform_datatypes.h>


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


    end_position_msg.header.frame_id = "/table";
    end_position_msg.pose.position.x = location_pose.position.x;
    end_position_msg.pose.position.y = location_pose.position.y;
    end_position_msg.pose.position.z = block.height/2.;
    end_position_msg.pose.orientation = tf::createQuaternionMsgFromYaw(0); // use no specific orientation but just yaw angle of zero

    srv.request.end_position = end_position_msg;

    ROS_INFO_STREAM("request: " << srv.request);

    bool success = false;

    if (client.call(srv))
    {
        ROS_INFO("Success: %d", (int)srv.response.success);

        if (srv.response.success)
        {
            success = update_block_pose(block_name, end_position_msg);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service kmr19_pick_up");
        return false;
    }

    if (!srv.response.success)
    {
        reasoning_planning::DispatchPlanFeedback msg;
        msg.success = false;

        msg.error_code = srv.response.error_code + 10;

        ROS_INFO_STREAM("PutDownAction: manipulation_response:\n" << srv.response);

        action_feedback_publisher.publish(msg);

    }
    else if (!success)
    {
        reasoning_planning::DispatchPlanFeedback msg;
        msg.success = false;
        msg.error_code = reasoning_planning::DispatchPlanFeedback::SERVICE_NOT_REACHABLE;
            
        ROS_INFO_STREAM("PutDownAction: manipulation_response:\n" << srv.response);

        action_feedback_publisher.publish(msg);

        return false;
    }
    else
    {
        // check if manipulation was successful based on perception	
        // scene db is already updated from above

        ros::Duration(2.).sleep(); // sleep 2 seconds to let perception enough time to process the new scene
        error_codes error = checkSceneDbAgainstPerception();
        if (error != NO_ERROR)
        {
            reasoning_planning::DispatchPlanFeedback msg;
            msg.success = false;

            switch (error)
            {
            case CANNOT_GET_SCENE_FROM_PERCEPTION:
            case CANNOT_GET_SCENE_FROM_SCENE_DB:
                msg.error_code = reasoning_planning::DispatchPlanFeedback::SERVICE_NOT_REACHABLE;
                break;
            case BLOCK_LISTS_NOT_SIMILAR:
                msg.error_code = reasoning_planning::DispatchPlanFeedback::PERCEPTION_DETECTED_DIFFERENT_POSITION_THAN_EXPECTED;
                break;
            default:
                msg.error_code = reasoning_planning::DispatchPlanFeedback::OTHER_ERROR;
                break;
            }

            ROS_INFO_STREAM("PutDownAction: manipulation_response:\n" << srv.response);

            action_feedback_publisher.publish(msg);

            return false;
        }
    }

    return success;
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