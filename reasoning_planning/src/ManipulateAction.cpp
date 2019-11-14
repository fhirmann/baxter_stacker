#include "ManipulateAction.h"

#include "mongodb_store/message_store.h"
#include "geometry_msgs/Pose.h"
#include <boost/foreach.hpp>

using namespace geometry_msgs;


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
