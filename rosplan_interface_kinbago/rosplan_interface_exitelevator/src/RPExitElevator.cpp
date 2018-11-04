#include "rosplan_interface_exitelevator/RPExitElevator.h"


/* The implementation of RPExitElevator.h
 * Based on the ideas and code from:
  * https://github.com/KCL-Planning/ROSPlan/blob/master/rosplan_interface_movebase/src/RPMoveBase.cpp
     (Source: Google Chrome source code https://github.com/KCL-Planning/ROSPlan/blob/master/rosplan_interface_movebase/src/RPMoveBase.cpp, March 10, 2016.)
 */

namespace KCL_rosplan
{
    /* constructor */
    RPExitElevator::RPExitElevator(ros::NodeHandle &nh, std::string &actionserver)
            : message_store(nh), action_client(actionserver, true)
    {
        // costmap client
        clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    }

    /* update barcode topic message*/
    void RPExitElevator::scanCallback(const std_msgs::String& msg)
    {
        barcode_string = msg;
    }

    /* run text to speech through external command */
    std::string RPExitElevator::runCommand(std::string cmd)
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

    /* get parameter ID from action dispatch */
    std::string RPExitElevator::getParameterId(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg, std::string parameter)
    {
        std::string parameterID="not found";

        for (size_t i = 0; i < msg->parameters.size(); i++)
        {
            if (0 == msg->parameters[i].key.compare(parameter))
                parameterID = msg->parameters[i].value;
        }

        return parameterID;
    }

    /* reminder to press the destination floor button*/
    void RPExitElevator::pressFloorButtonReminder(time_t startTime)
    {
        time_t currentTime;
        time (&currentTime);
        if (currentTime - startTime > 120)
        {
            runCommand(instructions);
            this->startTime = currentTime;
        }
    }


    /* wait for correct QR code*/
    void RPExitElevator::monitorQRcode(std::string floorID)
    {
        ros::Rate loop_rate(2);
        bool correct_qrcode = false;
        time (&startTime);

        while(!correct_qrcode)
        {
            // display waiting for QR code status
            ROS_INFO("KCL: (%s) waiting for the right floor: %s", params.name.c_str(), floorID.c_str());
            std::cout << barcode_string.data << std::endl;
            std::cout << floorID << std::endl;

            // reminder to press the floor button
            pressFloorButtonReminder(startTime);

            // exit loop if correct QR code found
            ros::spinOnce();
            if(0==barcode_string.data.compare(floorID))
            {
                std::cout << "floor found" << std::endl;
                correct_qrcode = true;
            }
            loop_rate.sleep();
        }

    }


    /* check if waypoint ID is in Pose */
    bool RPExitElevator::waypointIDinPose(std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results, std::string wpID)
    {
        if(results.size()<1)
        {
            ROS_INFO("KCL: (%s) aborting action dispatch; no matching wpID %s", params.name.c_str(), wpID.c_str());
            return false;
        }

        if(results.size()>1)
        {
            ROS_INFO("KCL: (%s) multiple waypoints share the same wpID", params.name.c_str());
        }

        ROS_INFO("KCL: (%s) waiting for move_base action server to start", params.name.c_str());
        action_client.waitForServer();

        return true;
    }

    /* dispatch the movebase action*/
    void RPExitElevator::dispatchMoveBase(std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results)
    {
        move_base_msgs::MoveBaseGoal goal;
        geometry_msgs::PoseStamped &pose = *results[0];
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = -1;
        goal.target_pose = pose;
        action_client.sendGoal(goal);
    }

    /* check if action was finished before timeout */
    bool RPExitElevator::finishedBeforeTimeout(bool finishedBeforeTimeout)
    {
        if (finishedBeforeTimeout)
        {
            actionlib::SimpleClientGoalState state = action_client.getState();
            ROS_INFO("KCL: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());

            if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                // publish feedback (achieved)
                return true;
            }

            else
            {
                // clear costmaps
                std_srvs::Empty emptySrv;
                clear_costmaps_client.call(emptySrv);

                // publish feedback (failed)
                return false;
            }
        }

        else
        {
            // timed out (failed)
            action_client.cancelAllGoals();
            ROS_INFO("KCL: (%s) action timed out", params.name.c_str());
            return false;
        }
    }


    /* action dispatch callback */
    bool RPExitElevator::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
    {
        // get destination floor ID from action dispatch
        std::string floorID = getParameterId(msg,"floor");
        if(0==floorID.compare("not found"))
        {
            ROS_INFO("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?floor", params.name.c_str());
            return false;
        }

        // provide elevator operation instructions
	int sleepTime = 4000000;
	usleep(sleepTime);
        std::stringstream ssInstructions;
        ssInstructions << "espeak -v en -s 150 \"Please press the floor " << floorID.at(2) << " button. Thanks!  \"";
        instructions = ssInstructions.str();
        runCommand(instructions);

        // wait for correct QR code
        // monitorQRcode(floorID);

        //  get waypoint ID from action dispatch
        std::string wpID = getParameterId(msg,"towp");
        if(0==wpID.compare("not found"))
        {
            ROS_INFO("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?toWp", params.name.c_str());
            return false;
        }

        // get pose from message store
        std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;

        // check if sceneDB querry is operational
        if(!message_store.queryNamed<geometry_msgs::PoseStamped>(wpID, results))
        {
            // no KMS connection (failed)
            ROS_INFO("KCL: (%s) aborting action dispatch; query to sceneDB failed", params.name.c_str());
            return false;
        }

        //check if waypoint ID is in Pose
        if (!waypointIDinPose(results, wpID))
        {
            return false;
        }

        // dispatch the movebase action
        dispatchMoveBase(results);

        // check if action was finished before timeout
        return finishedBeforeTimeout(action_client.waitForResult());

    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosplan_interface_exitelevator");
    ros::NodeHandle nh("~");

    std::string actionserver;
    nh.param("action_server", actionserver, std::string("/move_base"));

    // create PDDL action subscriber
    KCL_rosplan::RPExitElevator rpee(nh, actionserver);

    // subscribe to barcode topic
    ros::Subscriber barcode_sub = nh.subscribe("/barcode", 10, &KCL_rosplan::RPExitElevator::scanCallback, dynamic_cast<KCL_rosplan::RPExitElevator*>(&rpee));

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpee));
    rpee.runActionInterface();

    return 0;
}

