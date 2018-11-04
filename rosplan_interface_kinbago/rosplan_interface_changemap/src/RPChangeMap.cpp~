#include "rosplan_interface_changemap/RPChangeMap.h"
#include <std_msgs/Float64.h>

/* The implementation of RPChangeMap.h */
namespace KCL_rosplan {

	/* constructor */
	RPChangeMap::RPChangeMap(ros::NodeHandle &nh)
	 : message_store(nh) {

        initialpose_feedback_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, true);
		
	}

	/* action dispatch callback */
	bool RPChangeMap::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// get waypoint ID from action dispatch
		std::string wpID;
		bool found = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {

            std::string test;
            test = msg->parameters[i].key;
			ROS_INFO("KCL: (%s) changemap loop", test.c_str());

			if(0==msg->parameters[i].key.compare("towp")) {
				wpID = msg->parameters[i].value;
				found = true;
			}
		}

		if(!found) {
			ROS_INFO("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?toWp", params.name.c_str());
			return false;
		}

		// get pose from message store
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store.queryNamed<geometry_msgs::PoseStamped>(wpID, results)) {
			
			if(results.size()<1) {
				ROS_INFO("KCL: (%s) aborting action dispatch; no matching wpID %s", params.name.c_str(), wpID.c_str());
				return false;
			}
			if(results.size()>1) {
				ROS_INFO("KCL: (%s) multiple waypoints share the same wpID", params.name.c_str());
			}
			
		    // set new pose
            geometry_msgs::PoseStamped &poseStamped = *results[0];
            geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceStamped;
            poseWithCovarianceStamped.header.frame_id = "map";
            poseWithCovarianceStamped.pose.pose.position.x = poseStamped.pose.position.x;
            poseWithCovarianceStamped.pose.pose.position.y = poseStamped.pose.position.y;
            poseWithCovarianceStamped.pose.pose.position.z = 0.0;
            poseWithCovarianceStamped.pose.pose.orientation.x = 0.0;
            poseWithCovarianceStamped.pose.pose.orientation.y = 0.0;
            poseWithCovarianceStamped.pose.pose.orientation.z = 1.0;
            poseWithCovarianceStamped.pose.pose.orientation.w = 1.0;
            poseWithCovarianceStamped.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};

            // publish new pose - teleport robot
            initialpose_feedback_pub.publish(poseWithCovarianceStamped);
            ROS_INFO("KCL: (%s) initialpose published", params.name.c_str());

		}
        else{

            ROS_INFO("KCL: (%s) message_store.queryNamed error", params.name.c_str());

        }

	
	ROS_INFO("KCL: (%s) action finished", params.name.c_str());
		return true;


	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_changemap");
		ros::NodeHandle nh("~");  

		// create PDDL action subscriber
		KCL_rosplan::RPChangeMap rpcm(nh);

		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpcm));
		rpcm.runActionInterface();


		return 0;
	}
