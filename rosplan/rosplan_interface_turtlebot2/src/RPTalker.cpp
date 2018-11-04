#include "rosplan_interface_turtlebot/RPTalker.h"

/* The implementation of RPTalker.h
 * Based on the ideas and code from:
  * https://github.com/KCL-Planning/ROSPlan_interface_Turtlebot2/blob/master/src/RPTalker.cpp
		(Source: Google Chrome source code https://github.com/KCL-Planning/ROSPlan_interface_Turtlebot2/blob/master/src/RPTalker.cpp, March 10, 2016.)
*/
namespace KCL_rosplan 
{
	/* constructor */
	RPTalker::RPTalker(ros::NodeHandle &nh) 
	{
		// nothing yet...
		runCommand("espeak -v en -s 150 \"I am ready\"");
	}

	/* run text to speech through external command */
	std::string RPTalker::runCommand(std::string cmd)
	{
		std::string data;
		FILE *stream;
		char buffer[1000];
		stream = popen(cmd.c_str(), "r");
		while ( fgets(buffer, 1000, stream) != NULL )
			data.append(buffer);
		pclose(stream);
		return data;
	}

	/* get waypoint ID from action dispatch */
	std::string RPTalker::getWaypointId(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		std::string wpID;
		for (size_t i = 0; i < msg->parameters.size(); i++) 
		{
			if (0 == msg->parameters[i].key.compare("to")) 
				wpID = msg->parameters[i].value;
		}
		return wpID;
	}


	/* map string to int for switch */
	int RPTalker::actionNameToInt(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		int key = 0;
		std::map<int, std::string> actionsMap;
		actionsMap[1] = "goto_waypoint";
		actionsMap[2] = "enter_elevator";
		actionsMap[3] = "change_map";
		actionsMap[4] = "exit_elevator";
		actionsMap[5] = "greet_person";
		actionsMap[6] = "guide_person";
		actionsMap[7] = "dock";
		actionsMap[8] = "undock";
		actionsMap[9] = "localise";

		for (std::map<int, std::string>::iterator it = actionsMap.begin(); it != actionsMap.end(); ++it) 
		{
			if (0==msg->name.compare(it->second)) 
				key = it->first;
		}
		return key;
	}


	/* adjust espeak output from pddl action name to relevant output */
	std::string RPTalker::adjustActionName(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg, int key)
	{
		switch(key)
		{
			case 1: 
			{
				std::stringstream message;
				message << "going to waypoint " << getWaypointId(msg);
				return message.str();
			}
			case 2 :
				return "entering elevator";
			case 3 :
				return "changing map";
			case 4 :
				return "exiting elevator";
			case 5 :
				return "greeting_person";
			case 6 :
				return "destination reached";
			case 7 :
				return "docking";
			case 8 :
				return "undocking";
			case 9 :
				return "localizing";
		}
		return msg->name;
	}


	/* action dispatch callback */
	void RPTalker::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) 
	{
		// read out load the action name
		std::stringstream ss;
		ss << "espeak -v en -s 150 \"Turtlebot " << adjustActionName(msg,actionNameToInt(msg)) << "\"";
		runCommand(ss.str());
	}

	/* action talker callback */
	void RPTalker::talkerCallback(const std_msgs::String::ConstPtr& msg) 
	{
		// read out load the action name
		std::stringstream ss;
		ss << "espeak -v en -s 150 \"" << msg->data << "\"";
		runCommand(ss.str());
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) 
{
		ros::init(argc, argv, "rosplan_interface_talker");
		ros::NodeHandle nh;

		// create PDDL action subscriber
		KCL_rosplan::RPTalker rpta(nh);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPTalker::dispatchCallback, &rpta);
		ros::Subscriber ts = nh.subscribe("/kcl_rosplan/talker", 1000, &KCL_rosplan::RPTalker::talkerCallback, &rpta);
		ROS_INFO("KCL: (Talker) Ready to receive");

		ros::spin();
		return 0;
}

