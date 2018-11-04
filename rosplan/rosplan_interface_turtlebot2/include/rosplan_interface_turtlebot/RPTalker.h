#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <string>
#include <stdint.h>
#include <map>

#ifndef KCL_talker
#define KCL_talker

/**
 * This file defines the RPTalker class.
 * RPTalker is used to send strings to espeak.
 * It can state all action names as they are dispatched.
 * Based on the ideas and code from:
  * https://github.com/KCL-Planning/ROSPlan_interface_Turtlebot2/blob/master/include/rosplan_interface_turtlebot/RPTalker.h
		(Source: Google Chrome source code https://github.com/KCL-Planning/ROSPlan_interface_Turtlebot2/blob/master/include/rosplan_interface_turtlebot/RPTalker.h, March 10, 2016.)
*/
namespace KCL_rosplan {

	class RPTalker
	{

	private:
		/* run text to speech through external command */
		std::string runCommand(std::string cmd);

		/* get waypoint ID from action dispatch */
		std::string getWaypointId(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		/* map string to int for switch */
		int actionNameToInt(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		/* adjust espeak output from pddl action name to relevant output */
		std::string adjustActionName(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg, int key);

		

	public:

		/* constructor */
		RPTalker(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		void talkerCallback(const std_msgs::String::ConstPtr& msg);
	};
}
#endif

