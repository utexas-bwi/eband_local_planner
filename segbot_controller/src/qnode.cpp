/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <segbot_controller/qnode.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace segbot_controller {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{
    init();
    client.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true));   
  }

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
	wait();
}

void QNode::goalpub(const geometry_msgs::PoseStamped::ConstPtr msg) {
  //goal to send
  move_base_msgs::MoveBaseGoal newmovebasegoal;
  
  //get data from move_base_simple
  newmovebasegoal.target_pose.header = msg->header;
  newmovebasegoal.target_pose.pose = msg->pose;
  
  //publish MoveBaseGoal w/actionlib
  mostRecentGoal.reset(new move_base_msgs::MoveBaseGoal);
  *mostRecentGoal = newmovebasegoal;
  client->sendGoal(newmovebasegoal);
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"segbot_controller");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start();
  // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  sub.reset(new ros::Subscriber());
  *sub = n.subscribe("move_base_simple/interm_goal", 1000, &QNode::goalpub, this);
 
	start();
	return true;
}

void QNode::stop()
{
  client->cancelGoal();
}


void QNode::cont()
{
  client->sendGoal(*mostRecentGoal);
}


void QNode::run() {
  ros::spin();
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	//emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
		  ROS_DEBUG_STREAM(msg);
			logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
			break;
		}
		case(Info) : {
			ROS_INFO_STREAM(msg);
			logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
			break;
		}
		case(Warn) : {
			ROS_WARN_STREAM(msg);
			logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
			break;
		}
		case(Error) : {
			ROS_ERROR_STREAM(msg);
			logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
			break;
		}
		case(Fatal) : {
			ROS_FATAL_STREAM(msg);
			logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
			break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	//emit loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace segbot_controller
