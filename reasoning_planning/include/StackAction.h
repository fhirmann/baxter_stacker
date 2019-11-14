#include <ros/ros.h>
#include <vector>

#include "ManipulateAction.h"


#pragma once


class StackAction: public ManipulateAction
{

private:

public:

	/* constructor */
	StackAction(ros::NodeHandle &nh);

	/* listen to and process action_dispatch topic */
	bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};
