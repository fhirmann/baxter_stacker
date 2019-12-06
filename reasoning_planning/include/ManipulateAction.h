#pragma once


#include <ros/ros.h>

#include "rosplan_action_interface/RPActionInterface.h"


#include "geometry_msgs/Pose.h"

using namespace geometry_msgs;

#include "perception/Block.h"


class ManipulateAction: public KCL_rosplan::RPActionInterface
{

private:

public:

	ManipulateAction();

	/* listen to and process action_dispatch topic */
	bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) = 0;

protected:
	bool get_location_pose(std::string name, Pose& p);

	bool get_block_top_position(std::string name, Pose& p);
	bool get_block(std::string name, perception::Block& block);

	bool update_block_pose(std::string name, geometry_msgs::PoseStamped new_pose);

	bool get_where_on_table(std::string block_name, std::string& found_location_name);
	bool get_where_on(std::string block_name, std::string& found_block_bottom_name);

	static constexpr float s_WORLD_TO_TABLE_DISTANCE_Z = -0.2;
};
