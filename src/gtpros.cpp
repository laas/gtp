#include "gtp/gtpros.hpp"


//transfered in the CMakeLists.txt
//#define LIGHT_PLANNER
//#define GRASP_PLANNING
//#define USE_GSL
//#define USE_GBM
//#define MULTILOCALPATH
#include <libmove3d/planners/API/project.hpp>
#include <libmove3d/planners/GTP/taskManagerInterface.hpp>
#include <libmove3d/planners/Logging/Logger.h>
#include <libmove3d/planners/GTP/GTPTools/solutionTools/taskSolution.hpp>
#include <libmove3d/planners/GTP/GTPTools/solutionTools/gtpTrajectoryType.hpp>
#include <libmove3d/include/localpath.h>

#include <gtp_ros_msg/GTPTraj.h>
#include <algorithm>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

typedef Robot RobotDevice;

using namespace std;
using namespace toaster_msgs;

extern taskManagerInterface *TMI;

namespace move3d{

GtpRos::GtpRos(ros::NodeHandle *nh):
    _nh(nh),_nh_ws(0),_ws_cb_queue(0),_sc_mgr(0),_tmi(TMI),_updating(0),
    sync(0),object_sub(0),human_sub(0),robots_sub(0)
{
}

GtpRos::~GtpRos()
{
    _as->shutdown();
    delete _as;
    if (_ws_cb_queue) delete _ws_cb_queue;
    if(_nh_ws) delete _nh_ws;
    if(sync) delete sync;
    if(object_sub) delete object_sub;
    if(human_sub) delete human_sub;
    if(robots_sub) delete robots_sub;
}

void dummy_func(void){}

bool GtpRos::init()
{

    _sc_mgr = new SceneManager(_nh);
    _sc_mgr->fetchDofCorrespParam("/move3d/dof_name_corresp/PR2_ROBOT","PR2_ROBOT");

    _ws_cb_queue = new ros::CallbackQueue();
    _nh_ws = new ros::NodeHandle();
    _nh_ws->setCallbackQueue(_ws_cb_queue);

    _update_srv = _nh->advertiseService("/gtp/update",&GtpRos::updateSrvCb,this);
    _publishTraj_srv = _nh->advertiseService("/gtp/publishTraj",&GtpRos::publishTrajCb,this);
    _get_details_srv = _nh->advertiseService("/gtp/getDetails",&GtpRos::getDetailsCb,this);

    bool updateBaseOnly(0);
    _nh->param("/move3d/update_base_only",updateBaseOnly,false);
    _sc_mgr->setUpdateAcceptBaseOnly(updateBaseOnly);

    //initSubscriber();
    object_sub= new message_filters::Subscriber<toaster_msgs::ObjectListStamped> ();
    human_sub = new message_filters::Subscriber<toaster_msgs::HumanListStamped>  ();
    robots_sub= new message_filters::Subscriber<toaster_msgs::RobotListStamped>  ();
    sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10),*object_sub,*human_sub,*robots_sub);
    //sync->registerCallback(boost::bind(&GtpRos::worldUpdateCB,this,_1,_2,_3));
    sync->registerCallback(boost::bind(&GtpRos::worldUpdateCB,this,_1,_2,_3));

    _traj_pub = _nh->advertise<gtp_ros_msg::GTPTraj>("/gtp/trajectory",10);

    //init move3d
    logm3d::initializePlannerLogger();

    std::string p3d_file,sce_file;
    _nh->getParam("/move3d/p3dFile",p3d_file);
    _nh->getParam("/gtp/sceFile",sce_file);
    if(p3d_file.size()){
        _sc_mgr->setP3dPath(p3d_file);
        _sc_mgr->setScePath(sce_file);
        _sc_mgr->addModule("GTP");
        _sc_mgr->addModule("HriCostFunction");
        _sc_mgr->createScene();
    }else{
        ROS_FATAL("no /move3d/p3dFile param is set");
        return false;
    }

    _tmi=TMI;

    ext_g3d_draw_allwin_active = dummy_func; //TODO move this to scenemanager!?

    _saveScenarioSrv = new SaveScenarioSrv(_sc_mgr,_nh);
    _saveScenarioSrv->advertise("/gtp/save_scenario");

    _as=new actionlib::SimpleActionServer<gtp_ros_msgs::PlanAction>(*_nh,"gtp_server",boost::bind(&GtpRos::planCb,this,_1),false);
    _as->start();
    return true;
}

void GtpRos::run()
{
    ros::AsyncSpinner spinner(1),spinner2(1,_ws_cb_queue);
    spinner.start();
    spinner2.start();
    ros::waitForShutdown();
}

void GtpRos::planCb(const gtp_ros_msgs::PlanGoalConstPtr &request)
{
    using namespace gtp_ros_msgs;
    PlanResult result;
    PlanFeedback feedback;
    result.result.id.alternativeId=-1;
    result.result.id.taskId=-1;
    result.result.success=false;
    if(request->request.updateBefore && !_updating){
        triggerUpdate();
    }
    if(_updating){
        feedback.step="wait for update";
        _as->publishFeedback(feedback);
    }
    waitForUpdate();

    feedback.step="planning task";
    _as->publishFeedback(feedback);
    ROS_DEBUG("action planning request: %s",request->request.taskType.c_str());
    //hatp=ClearGTPInputs
    _tmi->clearAllInputs();
    //add agent
    foreach(const Role &ag,request->request.agents){
        _tmi->setAgent(ag.role,ag.name);
    }
    //add object
    foreach(const Role &ob,request->request.objects){
        _tmi->setObject(ob.role,ob.name);
    }
    //add data
    foreach(const MiscData &data,request->request.data){
        _tmi->addAdditinalData(data.key,data.value);
    }
    //add point
    foreach(const gtp_ros_msgs::Point &point,request->request.points){
        p3d_point val;
        val.x=point.point.x;
        val.y=point.point.y;
        val.z=point.point.z;
        _tmi->addPoint(point.key,val);
    }
    //plan task
    const string &type=request->request.taskType;
    int taskId=request->request.previousAction.taskId;
    int taskAltId=request->request.previousAction.alternativeId;
    bool computeMP=request->request.computeMotionPlan;

    WorldState *WS(0);

    if (taskId >= 0)
    {
        if (_tmi->getTaskManager()->getTask(taskId))
        {
            TaskSolution* TS = _tmi->getTaskManager()->getTask(taskId)->getResult()->getAlternativeWithId(taskAltId);
            if(TS)
            {
                WS = TS->getStopWS();
            }
        }
    }
    if (!WS && taskId == -1 && taskAltId == -1)
    {
        ROS_DEBUG("Ids -1 -1");
        WS = _tmi->getInitWS();
    }

    if (!WS)
    {
        ROS_DEBUG("no world state with this Ids, creating a new one from current");
        p3d_destroy_all_grasps();
        WS = new WorldState(global_Project->getActiveScene());
        _tmi->getTaskManager()->addWorldState(WS);
        WS->saveAll();
        WS->setAttachements(_tmi->getAttachements());
    }

    _tmi->setType(_tmi->getTaskManager()->getTaskIdFromString(type));
    _tmi->setTypeStr(type);

    _tmi->setWorldState(WS);
    _tmi->setComputeMP(computeMP);

    _tmi->callGTP();
    TaskSolution* TSol = _tmi->getCurrentSol();
    if(!TSol){
        feedback.step="no solution";
        _as->publishFeedback(feedback);
        ROS_DEBUG("failed to find a solution");
        result.result.success=false;
        result.result.status="no_solution";
    }else{
        feedback.step="post processing solution";
        _as->publishFeedback(feedback);
        result.result.id.taskId = TSol->getTask()->getId();
        result.result.id.alternativeId=TSol->getId();
        result.result.status="OK";
        result.result.success=true;
        vector<TaskSubSolution*> tss=TSol->getAllTSS();
        foreach(TaskSubSolution *ss,tss){
            gtp_ros_msgs::SubSolution subsol_msg;
            if(ss->getSubSolutionName() == "approach"){
                ROS_DEBUG("refining...");
                ss->refineTraj();
            }
            subsol_msg.agent = ss->getRobot()->getName();
            subsol_msg.armId= ss->getArmId();
            subsol_msg.id=ss->getId();
            subsol_msg.name=ss->getSubSolutionName();
            subsol_msg.type=GTPTrajectoryType::getInstance()->getTrajTypeAsString(ss->getSubSolutionType());

            result.result.solutionParts.push_back(subsol_msg);
        }
    }
    _as->setSucceeded(result);

    //_as->setAborted();
}

bool GtpRos::updateSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    if(_updating){
        resp.success=false;
        resp.message="Already updating";
        return true;
    }
    triggerUpdate();
    waitForUpdate();
    resp.success=true;
    return true;
}

bool GtpRos::cancelUpdateSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    _updating=false;
    destroyUpdateSubs();
    resp.success=true;
    resp.message="ok";
    return true;
}

bool GtpRos::setAttachmentCb(gtp_ros_msgs::SetAttachmentRequest &req, gtp_ros_msgs::SetAttachmentResponse &resp)
{
    if(!req.add_not_remove){
        ROS_INFO("removing all attachments");
        _tmi->clearAttachements();
        resp.success=true;
        resp.status="";
        return true;
    }

    //add attachment:

    RobotDevice *agent =  _scene->getRobotByName(req.agent);
    int armid=req.armId;
    if(!agent){
        ROS_ERROR("SetAttachment: agent %s not found",req.agent.c_str());
        resp.success=false;
        resp.status="agent not found";
        return true;
    }
    RobotDevice *object= _scene->getRobotByName(req.object);
    if(!object){
        ROS_ERROR("SetAttachment: object %s not found",req.agent.c_str());
        resp.success=false;
        resp.status="object not found";
        return true;
    }

    bool ok=_tmi->addAttachementFromObject(agent,object,armid);
    if(!ok){
        resp.success=false;
        resp.status="no grasp list";
        return true;
    }else{
        resp.success=true;
        resp.status="";
        return true;
    }

    return true;
}

bool GtpRos::setAttachmentFromTaskCb(gtp_ros_msgs::SetAttachmentFromTaskRequest &req, gtp_ros_msgs::SetAttachmentFromTaskResponse &resp)
{
    if(!req.add_not_remove){
        ROS_INFO("removing all attachments");
        _tmi->clearAttachements();
        resp.success=true;
        resp.status="";
        return true;
    }

    //set attachment:
    bool ok = _tmi->addAttachementsFromTask(req.actionId.taskId,req.actionId.alternativeId);
    if(!ok){
        resp.success=false;
        resp.status="no such task";
        return true;
    }else{
        resp.success=true;
        resp.status="";
        return true;
    }

}

void GtpRos::triggerUpdate()
{
    _updating=true;
    //sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10),*object_sub,*human_sub,*robots_sub);
    //sync->registerCallback(boost::bind(&GtpRos::worldUpdateCB,this,_1,_2,_3));
    //sync->registerCallback(boost::bind(&GtpRos::worldUpdateCB,this,_1,_2,_3));
    object_sub->subscribe(*_nh_ws,"/pdg/objectList",1);
    human_sub->subscribe(*_nh_ws,"/pdg/humanList",1);
    robots_sub->subscribe(*_nh_ws,"/pdg/robotList",1);
    ROS_DEBUG("Update triggered");
}

void GtpRos::waitForUpdate()
{
    if(!_updating) return;
    ROS_DEBUG("Wait for end of update");
    while(_updating){
        _ws_cb_queue->callAvailable();
        usleep(100);
    }
    ROS_DEBUG("Wait for update OK");
}

void GtpRos::destroyUpdateSubs()
{
    //disable
    //if(sync){
    //    delete sync;
    //}
    //sync=NULL;
    //if (human_sub)
    //    delete human_sub;
    //human_sub = NULL;
    //if (robots_sub)
    //    delete robots_sub;
    //robots_sub = NULL;
    //if (object_sub)
    //    delete object_sub;
    //object_sub = NULL;
    human_sub->unsubscribe();
    robots_sub->unsubscribe();
    object_sub->unsubscribe();

}

bool GtpRos::publishTrajCb(gtp_ros_msgs::PublishTrajRequest &req, gtp_ros_msgs::PublishTrajResponse &resp)
{
    Task *task=_tmi->getTaskManager()->getTask(req.actionId.taskId);
    if(!task){
        resp.status="action_id_not_found";
        resp.ok=false;
        return true;
    }
    TaskSolution *ts=task->getResult()->getAlternativeWithId(req.actionId.alternativeId);
    if(!ts){
        resp.status="alternative_id_not_found";
        resp.ok=false;
        return true;
    }
    TaskSubSolution *tss=ts->getSubSolutionWithId(req.subSolutionId);
    if(!tss){
        resp.status="subsolution_id_not_found";
        resp.ok=false;
        return true;
    }
    gtp_ros_msg::GTPTraj t_msg;
    t_msg.name="traj";
    RobotDevice *r=tss->getRobot();
    if(r->getHriAgent()->type==HRI_PR2){
        t_msg.traj.joint_names.push_back("navX");
        t_msg.traj.joint_names.push_back("navY");
        t_msg.traj.joint_names.push_back("dummyZ");
        t_msg.traj.joint_names.push_back("dummyRX");
        t_msg.traj.joint_names.push_back("dummyRY");
        t_msg.traj.joint_names.push_back("RotTheta");
        t_msg.traj.joint_names.push_back("torso_lift_joint");
        t_msg.traj.joint_names.push_back("head_pan_joint");
        t_msg.traj.joint_names.push_back("head_tilt_joint");
        t_msg.traj.joint_names.push_back("laser_tilt_mount_joint");

        t_msg.traj.joint_names.push_back("r_shoulder_pan_joint");
        t_msg.traj.joint_names.push_back("r_shoulder_lift_joint");
        t_msg.traj.joint_names.push_back("r_upper_arm_roll_joint");
        t_msg.traj.joint_names.push_back("r_elbow_flex_joint");
        t_msg.traj.joint_names.push_back("r_forearm_roll_joint");
        t_msg.traj.joint_names.push_back("r_wrist_flex_joint");
        t_msg.traj.joint_names.push_back("r_wrist_roll_joint");
        t_msg.traj.joint_names.push_back("r_gripper_joint");

        t_msg.traj.joint_names.push_back("dummyGripper");


        t_msg.traj.joint_names.push_back("l_shoulder_pan_joint");
        t_msg.traj.joint_names.push_back("l_shoulder_lift_joint");
        t_msg.traj.joint_names.push_back("l_upper_arm_roll_joint");
        t_msg.traj.joint_names.push_back("l_elbow_flex_joint");
        t_msg.traj.joint_names.push_back("l_forearm_roll_joint");
        t_msg.traj.joint_names.push_back("l_wrist_flex_joint");
        t_msg.traj.joint_names.push_back("l_wrist_roll_joint");
        t_msg.traj.joint_names.push_back("l_gripper_joint");

        t_msg.traj.joint_names.push_back("dummyLGripper");
    }

    int nb_dof = std::min<long unsigned int>(t_msg.traj.joint_names.size()+6,r->getRobotStruct()->nb_dof);
    if(tss->getSubSolutionType() == GTPTrajectoryType::Manipulate){
        p3d_traj *t=tss->getTraj();
        if(r && t && t->courbePt){
            localpath *lp = t->courbePt,*last_lp(0);
            while(lp){
                trajectory_msgs::JointTrajectoryPoint pt;
                configPt q=lp->config_at_param(r->getRobotStruct(),lp,0);
                for(int i=6;i<nb_dof;++i){
                    pt.positions.push_back(q[i]);
                }
                t_msg.traj.points.push_back(pt);

                last_lp=lp;
                lp=lp->next_lp;
            }
            trajectory_msgs::JointTrajectoryPoint pt;
            lp=last_lp;
            configPt q=lp->config_at_param(r->getRobotStruct(),lp,lp->range_param);
            for(int i=6;i<nb_dof;++i){
                pt.positions.push_back(q[i]);
            }
            t_msg.traj.points.push_back(pt);

            _traj_pub.publish(t_msg);
            resp.ok=true;
            resp.status="OK";
        }else{
            resp.ok=false;
            resp.status="no_traj_or_robot";
        }
    }else{
        ROS_DEBUG("not a Manipulate subtask %s",tss->getSubSolutionName().c_str());
        vector<p3d_point> navVect = tss->getNavVector();
        foreach(p3d_point p,navVect){
            trajectory_msgs::JointTrajectoryPoint pt;
            pt.positions.push_back(p.x);
            pt.positions.push_back(p.y);
            pt.positions.push_back(p.z);
        }
        _traj_pub.publish(t_msg);
        resp.ok=true;
        resp.status="OK";
    }
}

bool GtpRos::getDetailsCb(gtp_ros_msgs::GetDetailsRequest &req, gtp_ros_msgs::GetDetailsResponse &resp)
{
    Task *task=_tmi->getTaskManager()->getTask(req.actionId.taskId);
    if(!task){
        resp.status="action_id_not_found";
        resp.ok=false;
        return true;
    }
    TaskSolution *ts=task->getResult()->getAlternativeWithId(req.actionId.alternativeId);
    if(!ts){
        resp.status="alternative_id_not_found";
        resp.ok=false;
        return true;
    }

    vector<TaskSubSolution*> tss=ts->getAllTSS();
    foreach(TaskSubSolution *ss,tss){
        gtp_ros_msgs::SubSolution subsol_msg;
        subsol_msg.agent = ss->getRobot()->getName();
        subsol_msg.armId= ss->getArmId();
        subsol_msg.id=ss->getId();
        subsol_msg.name=ss->getSubSolutionName();
        subsol_msg.type=GTPTrajectoryType::getInstance()->getTrajTypeAsString(ss->getSubSolutionType());
        resp.solutionParts.push_back(subsol_msg);
    }
    resp.ok=true;
    resp.status="OK";
}


void GtpRos::worldUpdateCB(const ObjectListStampedConstPtr &object_list, const HumanListStampedConstPtr &human_list, const RobotListStampedConstPtr &robot_list){
    ROS_DEBUG("updating");
    foreach(const Object &o,object_list->objectList){
        _sc_mgr->updateObject(o.meEntity.name,o.meEntity.pose);
    }
    foreach(const toaster_msgs::Robot &r,robot_list->robotList){
        std::vector<double> q;
        for(unsigned int i=1;i<r.meAgent.skeletonJoint.size();++i){
            const toaster_msgs::Joint &jnt = r.meAgent.skeletonJoint[i];
            q.push_back(jnt.position);
        }
        bool ok = _sc_mgr->updateRobot(r.meAgent.meEntity.name,r.meAgent.meEntity.pose,q);
        if(!ok){
            //failure, try to provide the joint list name to scMgr
            ROS_DEBUG("try to provide the joint list name to scMgr");
            std::vector<std::string> names;
            for(unsigned int i=1;i<r.meAgent.skeletonNames.size();++i){
                const std::string &name = r.meAgent.skeletonNames[i];
                names.push_back(name);
            }
            _sc_mgr->setDofNameOrdered(r.meAgent.meEntity.name,names);
            _sc_mgr->updateRobot(r.meAgent.meEntity.name,r.meAgent.meEntity.pose,q);
        }
    }
    if(human_list->humanList.size()){
        ROS_WARN_ONCE("No support for humans yet. Cannot update human position");
        foreach (const toaster_msgs::Human &h, human_list->humanList) {
            ROS_DEBUG("Update human %s (%s)",h.meAgent.meEntity.name.c_str(),h.meAgent.meEntity.name.c_str());
            std::map<std::string, geometry_msgs::Pose> joints;
            for(uint i=0;i<h.meAgent.skeletonNames.size();++i){
                joints[h.meAgent.skeletonNames[i]]=h.meAgent.skeletonJoint[i].meEntity.pose;
            }
            _sc_mgr->updateHuman(h.meAgent.meEntity.name,h.meAgent.meEntity.pose,joints);
        }
    }

    _updated=true;
    _updating=false;
    //updated, delete the subscribers
    ROS_DEBUG("done updating");
    destroyUpdateSubs();
    //done, release mutex
    //_update_mutex.unlock();
}

}

