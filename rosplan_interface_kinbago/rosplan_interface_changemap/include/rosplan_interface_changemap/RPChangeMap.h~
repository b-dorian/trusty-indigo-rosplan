#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#include "rosplan_action_interface/RPActionInterface.h"

#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/Empty.h"

#ifndef KCL_movebase
#define KCL_movebase

/**
 * This file defines the RPChangeMap class.
 * RPChangeMap is used to change the pose of the robot (teleport) to another location on the map.
 * PDDL "change_map" action is listened for.
 * Waypoint goals are fetched by name from the SceneDB (implemented by mongoDB).
 */
namespace KCL_rosplan {

	class RPChangeMap: public RPActionInterface
	{

	private:

		mongodb_store::MessageStoreProxy message_store;
        ros::Publisher initialpose_feedback_pub;


	public:

		/* constructor */
		RPChangeMap(ros::NodeHandle &nh);     

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);  
	};
}
#endif
