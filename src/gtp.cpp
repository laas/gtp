//Copyright © 2017 CNRS-LAAS

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

#include "gtp/gtpros.hpp"

using namespace move3d;
int main (int argc,char **argv){
    ros::init(argc, argv, "gtp");
    ros::NodeHandle node;

    GtpRos gtp_ros(&node);
    if(gtp_ros.init()){
        gtp_ros.run();
    }else{
        return 1;
    }
    return 0;
}
