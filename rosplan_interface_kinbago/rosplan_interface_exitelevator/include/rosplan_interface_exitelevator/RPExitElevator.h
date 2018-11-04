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
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#ifndef KCL_movebase
#define KCL_movebase

/**
 * This file defines the RPExitElevator class.
 * RPExitElevator is used to start the elevator exit procedure.
 * PDDL "exit_elevator" action is listened for.
 * Waypoint goals are fetched by name from the SceneDB (implemented by mongoDB).
 * Based on the ideas and code from:
  * https://github.com/KCL-Planning/ROSPlan/blob/master/rosplan_interface_movebase/include/rosplan_interface_movebase/RPMoveBase.h
     (Source: Google Chrome source code https://github.com/KCL-Planning/ROSPlan/blob/master/rosplan_interface_movebase/include/rosplan_interface_movebase/RPMoveBase.h, March 10, 2016.)
 */
namespace KCL_rosplan 
{
    class RPExitElevator: public RPActionInterface
    {

    private:

        mongodb_store::MessageStoreProxy message_store;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client;
        ros::ServiceClient clear_costmaps_client;
        std_msgs::String barcode_string;
        std::string instructions;
	time_t startTime;
	
        /* run text to speech through external command */
        std::string runCommand(std::string cmd);

        /* get parameter ID from action dispatch */
        std::string getParameterId(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg, std::string parameter);

        /* wait for correct QR code*/
        void monitorQRcode(std::string floorID);

        /* reminder to press the destination floor button*/
        void pressFloorButtonReminder(time_t startTime);

        /* check if waypoint ID is in Pose */
        bool waypointIDinPose(std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results, std::string wpID);

        /* dispatch the movebase action*/
        void dispatchMoveBase(std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results);

        /* check if action was finished before timeout */
        bool finishedBeforeTimeout(bool finishedBeforeTimeout);



    public:

        /* constructor */
        RPExitElevator(ros::NodeHandle &nh, std::string &actionserver);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
        void scanCallback(const std_msgs::String& msg);
    };
}
#endif

