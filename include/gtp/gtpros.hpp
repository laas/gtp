#ifndef GTPROS_H
#define GTPROS_H

#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>

#include <move3d_ros_lib/scenemanager.h>
#include <move3d_ros_lib/savescenariosrv.h>

#include <gtp_ros_msgs/PlanAction.h>
#include <gtp_ros_msgs/PublishTraj.h>
#include <gtp_ros_msgs/GetDetails.h>
#include <gtp_ros_msgs/SetAttachment.h>
#include <gtp_ros_msgs/SetAttachmentFromTask.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <toaster_msgs/ObjectListStamped.h>
#include <toaster_msgs/HumanListStamped.h>
#include <toaster_msgs/RobotListStamped.h>

#include <boost/thread/mutex.hpp>

#include <std_srvs/Trigger.h>

class taskManagerInterface;
class Scene;
namespace move3d{
class GtpRos
{
public:
    GtpRos(ros::NodeHandle *nh);
    ~GtpRos();

    bool init();
    void run();

    void planCb(const gtp_ros_msgs::PlanGoalConstPtr &request);
    /**
     * @brief worldUpdateCB
     * @note clear attachments previously added with SetAttachment or SetAttachmentFromTask
     */
    void worldUpdateCB(const toaster_msgs::ObjectListStampedConstPtr &object_list, const toaster_msgs::HumanListStampedConstPtr &human_list, const toaster_msgs::RobotListStampedConstPtr &robot_list);
    bool updateSrvCb(std_srvs::TriggerRequest &req,std_srvs::TriggerResponse &resp);
    bool cancelUpdateSrvCb(std_srvs::TriggerRequest &req,std_srvs::TriggerResponse &resp);
    bool setAttachmentCb(gtp_ros_msgs::SetAttachmentRequest &req, gtp_ros_msgs::SetAttachmentResponse &resp);
    bool setAttachmentFromTaskCb(gtp_ros_msgs::SetAttachmentFromTaskRequest &req, gtp_ros_msgs::SetAttachmentFromTaskResponse &resp);
    void triggerUpdate();
    void waitForUpdate();
    void destroyUpdateSubs();
    bool publishTrajCb(gtp_ros_msgs::PublishTrajRequest &req,gtp_ros_msgs::PublishTrajResponse &resp);
    bool getDetailsCb(gtp_ros_msgs::GetDetailsRequest &req,gtp_ros_msgs::GetDetailsResponse &resp);

protected:
    ros::NodeHandle *_nh;
    ros::NodeHandle *_nh_ws;
    ros::CallbackQueue *_ws_cb_queue;
    actionlib::SimpleActionServer<gtp_ros_msgs::PlanAction> *_as;
    //gtp_ros_msgs::PlanResult _result;
    SceneManager *_sc_mgr;
    Scene *_scene;
    taskManagerInterface *_tmi;

    SaveScenarioSrv *_saveScenarioSrv;
    ros::ServiceServer _update_srv;
    ros::ServiceServer _cancel_update_srv;
    ros::ServiceServer _publishTraj_srv;
    ros::ServiceServer _get_details_srv;
    ros::ServiceServer _set_attach_task_srv;
    ros::ServiceServer _set_attach_obj_srv;
    ros::Publisher _traj_pub;

    typedef message_filters::sync_policies::ExactTime<toaster_msgs::ObjectListStamped,toaster_msgs::HumanListStamped,toaster_msgs::RobotListStamped> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> *sync;
    message_filters::Subscriber<toaster_msgs::ObjectListStamped> *object_sub;
    message_filters::Subscriber<toaster_msgs::HumanListStamped>  *human_sub;
    message_filters::Subscriber<toaster_msgs::RobotListStamped>  *robots_sub;

    bool _updating;
    bool _updated;

};
}

#endif // GTPROS_H
