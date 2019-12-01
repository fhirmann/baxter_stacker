#pragma once

#include "ManipulateAction.h"

class StackAction: public ManipulateAction
{

private:

public:

	/* constructor */
	StackAction(ros::NodeHandle &nh);

	/* listen to and process action_dispatch topic */
	bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};
