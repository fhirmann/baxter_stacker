#include "ManipulateAction.h"

#include "mongodb_store/message_store.h"
#include <boost/foreach.hpp>

#include "rosplan_knowledge_msgs/GetAttributeService.h"

#include "rosplan_knowledge_msgs/KnowledgeItem.h"

#include "diagnostic_msgs/KeyValue.h"

using namespace geometry_msgs;


ManipulateAction::ManipulateAction()
{
    std::string service_names[] = {"/rosplan_knowledge_base/state/propositions", 
                                "kmr19_manipulation_pick_up",
                                "kmr19_manipulation_put_down"};

    for (auto service_name : service_names)
    {
        ROS_INFO("waiting for service %s",service_name.c_str());
        ros::service::waitForService(service_name);
        ROS_INFO("service %s is now running",service_name.c_str());
    } 

    ros::NodeHandle nh;

    action_feedback_publisher = nh.advertise<reasoning_planning::DispatchPlanFeedback>("action_feedback",100);


}


/**
 * @brief returns the pose of a location
 * 
 * @param name the name of the location
 * @param p output: the pose of the location
 * @return true if found
 * @return false if not found or multiple locations with the same name
 */
bool ManipulateAction::get_location_pose(std::string name, Pose& p)
{
    ros::NodeHandle nh;
    mongodb_store::MessageStoreProxy messageStore(nh);    

    std::vector< boost::shared_ptr<Pose> > results;

    if(!messageStore.queryNamed<Pose>(name, results))
    {
        ROS_ERROR("Location for \"%s\" could not be retrieved. Either there is a database error or there is no location pose added in the scene database", name.c_str());
        return false;
    }

    if (results.size() == 0)
    {
        ROS_ERROR("There is no location \"%s\" stored in the scene database", name.c_str());
        return false;
    }
    else if (results.size() != 1)
    {
        ROS_ERROR("There is more than one location \"%s\" stored in the scene database (size=%zu)", name.c_str(), results.size());
        return false;
    }

    p = *(results.at(0));

    return true;
}

/**
 * @brief returns the location instance name where the block is 'on_table'.  
 * 
 * @param block_name the block name which is checked if 'on_table'
 * @param found_block_bottom_name output: the found location name where the block is on the table
 * @return true if this block is 'on_table' at any location
 * @return false otherwise or if a needed service cannot be called
 */
bool ManipulateAction::get_where_on_table(std::string block_name, std::string& found_location_name)
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/propositions");
    

    rosplan_knowledge_msgs::GetAttributeService srv;


    srv.request.predicate_name = "on_table";

    ROS_INFO_STREAM("request: " << srv.request);

    if (!client.call(srv))
    {
        ROS_ERROR("Could not call service \"/rosplan_knowledge_base/state/propositions\"");
        return false;
    }

    ROS_INFO_STREAM("response: " << srv.response);

    for(auto item : srv.response.attributes)
    {
        auto key_values = item.values;

        if (key_values[0].value.compare(block_name) == 0)
        {
            found_location_name = key_values[1].value;
            return true;
        }
    }

    return false;
}

/**
 * @brief returns the block instance name where the block is 'on'.  
 * 
 * @param block_name the block name which is checked if 'on' some other block
 * @param found_block_bottom_name output: the found block name which is below 'block_name'
 * @return true if this block is 'on' any block
 * @return false otherwise or if a needed service cannot be called
 */
bool ManipulateAction::get_where_on(std::string block_name, std::string& found_block_bottom_name)
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/propositions");

    rosplan_knowledge_msgs::GetAttributeService srv;


    srv.request.predicate_name = "on";

    ROS_INFO_STREAM("request: " << srv.request);

    if (!client.call(srv))
    {
        ROS_ERROR("Could not call service \"/rosplan_knowledge_base/state/propositions\"");
        return false;
    }

    ROS_INFO_STREAM("response: " << srv.response);

    for(auto item : srv.response.attributes)
    {
        auto key_values = item.values;

        if (key_values[0].value.compare(block_name) == 0)
        {
            found_block_bottom_name = key_values[1].value;
            return true;
        }
    }

    return false;
}

/**
 * @brief return the center of the top face of the block
 * 
 * @param name the name of the block in the scene db
 * @param p output: the pose of the center of the top face of the block
 * @return true if found
 * @return false if not found or multiple blocks with the same name
 */
bool ManipulateAction::get_block_top_position(std::string name, Pose& p)
{

    float height_from_top_face_to_table = 0;
    std::string current_block_name = name;
    std::string found_location_name = "";
    std::string found_block_bottom_name = "";

    while(!get_where_on_table(current_block_name, found_location_name))
    {
        if (!get_where_on(current_block_name, found_block_bottom_name))
        {
            // The block is not on the table and not on some other block -> it must be some other major database inconsistency
            ROS_ERROR("The block %s is not on the table and not on some other block -> it must be some other major database inconsistency", current_block_name.c_str());
            return false;
        }

        perception::Block block;
        if (!get_block(current_block_name, block))
        {
            return false;
        }

        height_from_top_face_to_table += block.height;

        current_block_name = found_block_bottom_name;

    }

    perception::Block block;
    if (!get_block(current_block_name, block))
    {
        return false;
    }

    height_from_top_face_to_table += block.height;

    if (!get_location_pose(found_location_name, p))
    {
        return false;
    }

    p.position.z = height_from_top_face_to_table;


    return true;
}

/**
 * @brief return the block stored with this name stored in the scene db
 * 
 * @param name the name of the block in the scene db
 * @param p output: the block message describing the block
 * @return true if found
 * @return false if not found or multiple blocks with the same name
 */
bool ManipulateAction::get_block(std::string name, perception::Block& block)
{

    ros::NodeHandle nh;
    mongodb_store::MessageStoreProxy messageStore(nh);    

    std::vector< boost::shared_ptr<perception::Block> > results;

    if(!messageStore.queryNamed<perception::Block>(name, results))
    {
        ROS_ERROR("Block for \"%s\" could not be retrieved. Either there is a database error or there is no block added in the scene database", name.c_str());
        return false;
    }

    if (results.size() == 0)
    {
        ROS_ERROR("There is no block \"%s\" stored in the scene database", name.c_str());
        return false;
    }
    else if (results.size() != 1)
    {
        ROS_ERROR("There is more than one block \"%s\" stored in the scene database (size=%zu)", name.c_str(), results.size());
        return false;
    }

    block = *(results.at(0));

    return true;
}


bool ManipulateAction::update_block_pose(std::string name, geometry_msgs::PoseStamped new_pose)
{

    ros::NodeHandle nh;
    mongodb_store::MessageStoreProxy messageStore(nh);    

    perception::Block block;

    if(!get_block(name, block))
    {
        return false;
    }

    block.pose = new_pose;

    if(!messageStore.updateNamed(name, block))
    {
        ROS_ERROR("Block for \"%s\" could not be updated. Either there is a database error or there is no block added in the scene database", name.c_str());
        return false;
    }

    return true;
}