#include <ros/ros.h>
#include <vector>

#include "ManipulateAction.h"


#pragma once


class UnstackAction: public ManipulateAction
{

private:

public:

	/* constructor */
	UnstackAction(ros::NodeHandle &nh);

	/* listen to and process action_dispatch topic */
	bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};
