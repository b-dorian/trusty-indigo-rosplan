#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "geometry_msgs/Twist.h"

/**
 * This file defines the RPPersonInteraction class.
 * RPPersonInteraction is used to greet and signal destination reach to the pearson
 * PDDL "greet_person" and "guide_person" actions are listened for.
 */
namespace KCL_rosplan {

    class RPPersonInteraction
    {

    private:

        ros::ServiceClient update_knowledge_client;
        ros::Publisher action_feedback_pub;
        ros::Publisher cmd_vel_pub;
        std::string name;


        std::string runCommand(std::string cmd);
        void publishFeedback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg, std::string action_status,  rosplan_dispatch_msgs::ActionFeedback fb);
        void addPredicate(rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv, diagnostic_msgs::KeyValue pair, std::string predicate);

    public:

        /* constructor */
        RPPersonInteraction(ros::NodeHandle &nh);

        /* listen to and process action_dispatch topic */
        void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
    };
}

