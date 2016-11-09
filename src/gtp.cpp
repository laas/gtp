#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <ros/ros.h>
#include <signal.h>
#include <log4cxx/logmanager.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <toaster_msgs/Fact.h>
#include <toaster_msgs/FactList.h>
#include <toaster_msgs/ObjectListStamped.h>
#include <toaster_msgs/RobotListStamped.h>
#include <toaster_msgs/HumanListStamped.h>

#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>

//transfered in the CMakeLists.txt
//#define LIGHT_PLANNER
//#define GRASP_PLANNING
//#define USE_GSL
//#define USE_GBM
//#define MULTILOCALPATH
#include <libmove3d/planners/API/project.hpp>
#include <libmove3d/planners/GTP/taskManagerInterface.hpp>
#include <libmove3d/planners/Logging/Logger.h>

#include <tr1/shared_ptr.h>

#include <move3d_ros_lib/scenemanager.h>
#include <move3d_ros_lib/savescenariosrv.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>


int main (int argc,char **argv){
    ros::init(argc, argv, "gtp");
    ros::NodeHandle node;

}
