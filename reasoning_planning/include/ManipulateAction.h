#pragma once


#include <ros/ros.h>

#include "rosplan_action_interface/RPActionInterface.h"


#include "geometry_msgs/Pose.h"

using namespace geometry_msgs;

#include "perception/Block.h"


#include "reasoning_planning/DispatchPlanFeedback.h"

#include <cmath>


class ManipulateAction: public KCL_rosplan::RPActionInterface
{

private:

public:

	ManipulateAction();

	/* listen to and process action_dispatch topic */
	bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) = 0;

	enum error_codes
	{
		NO_ERROR = 0,
		CANNOT_GET_SCENE_FROM_PERCEPTION = 1,
		CANNOT_GET_SCENE_FROM_SCENE_DB = 2,
		BLOCK_LISTS_NOT_SIMILAR = 3
	};

protected:
	bool get_location_pose(std::string name, Pose& p);

	bool get_block_top_position(std::string name, Pose& p);
	bool get_block(std::string name, perception::Block& block);

	bool update_block_pose(std::string name, geometry_msgs::PoseStamped new_pose);

	bool get_where_on_table(std::string block_name, std::string& found_location_name);
	bool get_where_on(std::string block_name, std::string& found_block_bottom_name);

	bool getPerceptionSceneBlockList(std::vector<perception::Block> &block_list);
	bool getSceneDbBlockList(std::vector<perception::Block> &block_list);

	bool checkIfBlocksListsSimilar(std::vector<perception::Block> &block_list_1, std::vector<perception::Block> &block_list_2);

	// same values as taken in reasoning_planning_interface.py
    static constexpr double EPSILON = 0.06;

	bool is_epsilon_close(double val1, double val2, double epsilon = EPSILON){ return std::abs(val1 - val2) < epsilon; }

	double getEuclideanDistance(perception::Block block1, perception::Block block2);

	
	error_codes checkSceneDbAgainstPerception();

	
	ros::Publisher action_feedback_publisher;
};
