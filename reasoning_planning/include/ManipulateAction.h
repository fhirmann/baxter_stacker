#include <ros/ros.h>
#include <vector>

#include "rosplan_action_interface/RPActionInterface.h"


#pragma once


#include "geometry_msgs/Pose.h"

using namespace geometry_msgs;


class ManipulateAction: public KCL_rosplan::RPActionInterface
{

private:

public:

	/* listen to and process action_dispatch topic */
	bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) = 0;

protected:
	bool get_location_pose(std::string name, Pose& p);
};
