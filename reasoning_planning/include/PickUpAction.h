#include <ros/ros.h>
#include <vector>

#include "ManipulateAction.h"


#pragma once


class PickUpAction: public ManipulateAction
{

private:

public:

	/* constructor */
	PickUpAction(ros::NodeHandle &nh);

	/* listen to and process action_dispatch topic */
	bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};