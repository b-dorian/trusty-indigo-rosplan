#include "rosplan_interface_enterelevator/RPEnterElevator.h"

/* The implementation of RPEnterElevator.h
 * Based on the ideas and code from:
  * https://github.com/KCL-Planning/ROSPlan/blob/master/rosplan_interface_movebase/src/RPMoveBase.cpp
     (Source: Google Chrome source code https://github.com/KCL-Planning/ROSPlan/blob/master/rosplan_interface_movebase/src/RPMoveBase.cpp, March 10, 2016.)
 */
namespace KCL_rosplan
{
    /* constructor */
    RPEnterElevator::RPEnterElevator(ros::NodeHandle &nh, std::string &actionserver)
            : message_store(nh), action_client(actionserver, true)
    {
        // costmap client
        clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

        // elevator call instructions
        instructions = "espeak -v en -s 150 \"Please call the elevator. I am programmed to only work with the middle one. Thanks!  \"";
    }


    /* update laser scan topic message*/
    void RPEnterElevator::scanCallback(const sensor_msgs::LaserScan& msg)
    {
        last_scan = msg;
    }

    /* text to speech */
    std::string RPEnterElevator::runCommand(std::string cmd)
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


    /* reminder to call the middle elevator*/
    void RPEnterElevator::callElevatorReminder(time_t startTime)
    {
        time_t currentTime;
        time (&currentTime);
        if (currentTime - startTime > 120)
        {
            runCommand(instructions);
            this->startTime = currentTime;
        }
    }


    /* wait for elevator doors to open*/
    void RPEnterElevator::monitorDoor()
    {
        ros::Rate loop_rate(2);
        bool door_open = false;
        time (&startTime);

        while(!door_open)
        {
            // display waiting for door open status
            ROS_INFO("KCL: (%s) waiting for door to open", params.name.c_str());
            std::cout << last_scan.ranges[last_scan.ranges.size()/2] << std::endl;

            // reminder to call the middle elevator
            callElevatorReminder(startTime);

            // exit loop if door opened
            ros::spinOnce();
            if(last_scan.ranges[last_scan.ranges.size()/2] > 1.2)
                door_open = true;
            loop_rate.sleep();
        }

    }

    /* get waypoint ID from action dispatch */
    std::string RPEnterElevator::getWaypointId(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
    {
        std::string wpID="not found";

        for (size_t i = 0; i < msg->parameters.size(); i++)
        {
            if (0 == msg->parameters[i].key.compare("towp"))
                wpID = msg->parameters[i].value;
        }

        return wpID;
    }

    /* check if waypoint ID is in Pose */
    bool RPEnterElevator::waypointIDinPose(std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results, std::string wpID)
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
    void RPEnterElevator::dispatchMoveBase(std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results)
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
    bool RPEnterElevator::finishedBeforeTimeout(bool finishedBeforeTimeout)
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
    bool RPEnterElevator::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        // instruct the person to call the elevator
	int sleepTime = 4000000;
	usleep(sleepTime);
        runCommand(instructions);

        //  wait for elevator doors to open
        //  monitorDoor();

        //  get waypoint ID from action dispatch
        std::string wpID = getWaypointId(msg);
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

    ros::init(argc, argv, "rosplan_interface_enterelevator");
    ros::NodeHandle nh("~");

    std::string actionserver;
    nh.param("action_server", actionserver, std::string("/move_base"));

    // create PDDL action subscriber
    KCL_rosplan::RPEnterElevator rpee(nh, actionserver);

    // subscribe to /scan topic
    ros::Subscriber ls = nh.subscribe("/scan", 10, &KCL_rosplan::RPEnterElevator::scanCallback, dynamic_cast<KCL_rosplan::RPEnterElevator*>(&rpee));

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpee));

    rpee.runActionInterface();

    return 0;
}

