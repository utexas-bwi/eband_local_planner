/**
 * @file /include/segbot_controller/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef segbot_controller_QNODE_HPP_
#define segbot_controller_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace segbot_controller {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
	void stopmsg(const geometry_msgs::PoseStamped::ConstPtr msg);
  void goalpub(const geometry_msgs::PoseStamped::ConstPtr msg);

	/*********************
	** Logging
	**********************/
	enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
public slots:
  void stop();
  void cont();


signals:
  void loggingUpdated();
  void rosShutdown();

public:
  int init_argc;
  char** init_argv;
  boost::shared_ptr<ros::Subscriber> sub;
  QStringListModel logging_model;
  boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > client; 
  boost::shared_ptr<move_base_msgs::MoveBaseGoal> mostRecentGoal;
};

}  // namespace segbot_controller

#endif /* segbot_controller_QNODE_HPP_ */
