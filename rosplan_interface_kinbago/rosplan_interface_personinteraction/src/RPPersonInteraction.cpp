#include "rosplan_interface_personinteraction/RPPersonInteraction.h"

/* The implementation of RPPersonInteraction.h
 * Based on the ideas and code from:
  * https://github.com/KCL-Planning/ROSPlan_interface_Turtlebot2/blob/master/src/RPDocker.cpp
		(Source: Google Chrome source code https://github.com/KCL-Planning/ROSPlan_interface_Turtlebot2/blob/master/src/RPDocker.cpp, March 10, 2016.)
*/

namespace KCL_rosplan {

    /* constructor */
    RPPersonInteraction::RPPersonInteraction(ros::NodeHandle &nh){

        // knowledge interface
        update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

        // create publishers
        action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10, true);
    }


    // text to speech
    std::string RPPersonInteraction::runCommand(std::string cmd)
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

    // publish feedback
    void RPPersonInteraction::publishFeedback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg, std::string action_status, rosplan_dispatch_msgs::ActionFeedback fb)
    {
        fb.action_id = msg->action_id;
        fb.status = action_status;
        action_feedback_pub.publish(fb);
    }

    // add predicate
    void RPPersonInteraction::addPredicate(rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv, diagnostic_msgs::KeyValue pair, std::string predicate)
    {
        updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
        updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        updatePredSrv.request.knowledge.attribute_name = predicate;
        pair.key = "v";
        pair.value = name;
        updatePredSrv.request.knowledge.values.push_back(pair);
        update_knowledge_client.call(updatePredSrv);
    }

    /* action dispatch callback */
    void RPPersonInteraction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        rosplan_dispatch_msgs::ActionFeedback fb;
        rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
        diagnostic_msgs::KeyValue pair;
        int sleepTime = 4000000;

        // greet_person action
        if(0==msg->name.compare("greet_person")) {

            ROS_INFO("KCL: (greet_person) action received");

            usleep(sleepTime);

            // greet the person
            runCommand("espeak -v en -s 150 \"Hi, my name is Dorian, and I will be your guide today. You will have to take care of pressing the elevator buttons. And open all the doors along the way. The rest is up to me. Just follow me and I will take you to your destination. \"");

            // publish feedback (enabled)
            publishFeedback(msg, "action enabled", fb);

            // add predicates
            addPredicate(updatePredSrv, pair, "person_greeted");
            addPredicate(updatePredSrv, pair, "allowed_goto_waypoint");

            ros::Rate big_rate(0.5);
            big_rate.sleep();

            // publish feedback (achieved)
            publishFeedback(msg, "action achieved", fb);

            ROS_INFO("KCL: (greet_person) action complete");

        }

            // guide_person action
        else if(0==msg->name.compare("guide_person")) {

            ROS_INFO("KCL: (guide_person) action recieved");

            usleep(sleepTime);

            // inform the person that the destination was reached and say good bye
            runCommand("espeak -v en -s 150 \"You have reached your destination. I will go back to the reception, bye bye. \"");

            usleep(sleepTime);

            // publish feedback (enabled)
            publishFeedback(msg, "action enabled", fb);

            // add predicates
            addPredicate(updatePredSrv, pair, "person_guided");
            addPredicate(updatePredSrv, pair, "allowed_goto_waypoint");

            ros::Rate big_rate(0.5);
            big_rate.sleep();

            // publish feedback (achieved)
            publishFeedback(msg, "action achieved", fb);

            ROS_INFO("KCL: (guide_person) action complete");
        }

    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "rosplan_interface_personinteraction");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    KCL_rosplan::RPPersonInteraction rppi(nh);

    // listen for action dispatch
    ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPPersonInteraction::dispatchCallback, &rppi);
    ROS_INFO("KCL: (person interaction) Ready to receive");

    ros::spin();
    return 0;
}
