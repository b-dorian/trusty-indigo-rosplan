#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <ctime>

#include "rosplan_action_interface/RPActionInterface.h"

#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/LaserScan.h"

#ifndef KCL_movebase
#define KCL_movebase

/**
 * This file defines the RPEnterElevator class.
 * RPExirElevator is used to start the elevator enter procedure.
 * PDDL "enter_elevator" action is listened for.
 * Waypoint goals are fetched by name from the SceneDB (implemented by mongoDB).
 * Based on the ideas and code from:
  * https://github.com/KCL-Planning/ROSPlan/blob/master/rosplan_interface_movebase/include/rosplan_interface_movebase/RPMoveBase.h
     (Source: Google Chrome source code https://github.com/KCL-Planning/ROSPlan/blob/master/rosplan_interface_movebase/include/rosplan_interface_movebase/RPMoveBase.h, March 10, 2016.)
 */
namespace KCL_rosplan 
{
    class RPEnterElevator: public RPActionInterface
    {

    private:

        mongodb_store::MessageStoreProxy message_store;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client;
        ros::ServiceClient clear_costmaps_client;
        sensor_msgs::LaserScan last_scan;
        std::string instructions;
	time_t startTime;


	/* run text to speech through external command */
        std::string runCommand(std::string cmd);

	/* wait for elevator doors to open*/
        void monitorDoor();

	/* reminder to call the middle elevator*/
        void callElevatorReminder(time_t startTime);

	/* get waypoint ID from action dispatch */
        std::string getWaypointId(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

	/* check if waypoint ID is in Pose */
        bool waypointIDinPose(std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results, std::string wpID);

	/* dispatch the movebase action*/
        void dispatchMoveBase(std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results);

	/* check if action was finished before timeout */
        bool finishedBeforeTimeout(bool finishedBeforeTimeout);

    public:

        /* constructor */
        RPEnterElevator(ros::NodeHandle &nh, std::string &actionserver);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
        void scanCallback(const sensor_msgs::LaserScan& msg);
    };
}
#endif

