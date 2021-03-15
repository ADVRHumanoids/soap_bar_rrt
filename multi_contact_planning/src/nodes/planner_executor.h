#ifndef PLANNER_EXECUTOR_H
#define PLANNER_EXECUTOR_H

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <XBotInterface/ModelInterface.h>

#include <cartesian_interface/utils/RobotStatePublisher.h>

#include "constraints/cartesian_constraint.h"
#include "planner/cartesio_ompl_planner.h"
#include "utils/robot_viz.h"
#include "validity_checker/validity_predicate_aggregate.h"
#include "validity_checker/collisions/planning_scene_wrapper.h"
#include "validity_checker/validity_checker_context.h"

#include "multi_contact_planning/CartesioPlanner.h"

#include "nodes/goal_generation.h"
// #include "goal/NSPG.h"

#include "utils/point_cloud_manager.h" //LR

#include "planner/cartesian_trajectory_interpolation.h"

#include "planner/multi_contact/Configuration.hpp"
#include "planner/multi_contact/enum.h"

#include "planner/multi_contact/Planner.hpp" //PF
#include "planner/multi_contact/utils.hpp" //PF
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <thread>
#include <iostream>
#include <string>

namespace XBot { namespace Cartesian {

class PlannerExecutor
{

public:

    PlannerExecutor();
    PlannerExecutor(const PlannerExecutor&) = delete;
    PlannerExecutor& operator=(const PlannerExecutor&) = delete;

    void run();

    void setStartState(const JointNameMap &q);
    void setGoalState(const JointNameMap &q);

    /**
     * @brief callPlanner
     * @param time
     * @param planner_type
     * @param interpolation_time is the time used for interpolation
     * @param trajectory interpolated trajectory from planner
     * @return enum StatusType
            {
                /// Uninitialized status
                UNKNOWN = 0,
                /// Invalid start state or no start state specified
                INVALID_START,
                /// Invalid goal state
                INVALID_GOAL,
                /// The goal is of a type that a planner does not recognize
                UNRECOGNIZED_GOAL_TYPE,
                /// The planner failed to find a solution
                TIMEOUT,
                /// The planner found an approximate solution
                APPROXIMATE_SOLUTION,
                /// The planner found an exact solution
                EXACT_SOLUTION,
                /// The planner crashed
                CRASH,
                /// The planner did not find a solution for some other reason
                ABORT,
                /// The number of possible status values
                TYPE_COUNT
            };
     */
    int callPlanner(const double time, const std::string& planner_type, const double interpolation_time, const double goal_thrs,
                    std::vector<Eigen::VectorXd>& trajectory);
    /**
     * @param base_distal_links couple of base_link and distal_link to compute Cartesian trajectory
     * @param cartesian_trajectories vector of Cartesian trajactories (each element correspond to couple base_link, distal_link)
     */
    int callPlanner(const double time, const std::string& planner_type, const double interpolation_time, const double goal_thrs,
                    const std::vector<std::pair<std::string, std::string> > base_distal_links,
                    std::vector<Eigen::VectorXd>& trajectory,
                    std::vector<std::vector<Eigen::Affine3d> >& cartesian_trajectories);

private:

    /////////////////////////////////////////// PF
    //std::vector<Stance> sigmaList;
    //std::vector<Configuration> qList;
    //bool solFound;

    int index_config;
    std::vector<Eigen::VectorXd> plan;

    int n_dof;
    Eigen::VectorXd q_init;
    Eigen::VectorXd q_goal;
    
    std::vector<EndEffector> activeEEsInit;
    std::vector<EndEffector> activeEEsGoal;
    std::vector<EndEffector> allowedEEs;

    /////////////////////////////////////////// PF
    
    typedef std::shared_ptr<Planning::PlanningSceneWrapper> PlanningScenePtr;

    void init_load_model();
    void init_load_config();
    void init_load_planner();
    void init_load_validity_checker();
    void init_subscribe_start_goal();
    void init_trj_publisiher();
    void init_planner_srv();
    void init_goal_generator();
    void init_interpolator();

    // PF ///////////////////////////////////////////////////////////////////////
    void setReferences(std::vector<std::string> active_tasks, std::vector<Eigen::Affine3d> ref_tasks, Eigen::VectorXd q_ref);
    void writeOnFileConfigs(std::vector<Configuration> qList, std::string fileName);
    void writeOnFileStances(std::vector<Stance> sigmaList, std::string fileName);
    void readFromFileConfigs(std::vector<Configuration> &qList, std::string fileName);
    void readFromFileStances(std::vector<Stance> &sigmaList, std::string fileName);
    /////////////////////////////////////////////////////////////////////////////

    void on_start_state_recv(const sensor_msgs::JointStateConstPtr& msg);
    void on_goal_state_recv(const sensor_msgs::JointStateConstPtr& msg);
    bool planner_service(multi_contact_planning::CartesioPlanner::Request& req,
                         multi_contact_planning::CartesioPlanner::Response& res);
    bool get_planning_scene_service(moveit_msgs::GetPlanningScene::Request& req,
                                    moveit_msgs::GetPlanningScene::Response& res);
    bool apply_planning_scene_service(moveit_msgs::ApplyPlanningScene::Request& req,
                                      moveit_msgs::ApplyPlanningScene::Response& res);
    bool goal_sampler_service(multi_contact_planning::CartesioGoal::Request& req,
                              multi_contact_planning::CartesioGoal::Response& res);
    
    //Eigen::Matrix3d generateRotationAroundAxis(EndEffector pk, Eigen::Vector3d axis);



    Planning::CartesianConstraint::Ptr make_manifold(std::string problem_description_string);
    bool check_state_valid(XBot::ModelInterface::ConstPtr model);
    void publish_and_check_start_and_goal_models(ros::Time time);
    void enforce_bounds(Eigen::VectorXd& q) const;

    ros::NodeHandle _nh, _nhpr, _n;
    YAML::Node _planner_config;
    XBot::ModelInterface::Ptr _model;;
    Planning::OmplPlanner::Ptr _planner;
    Planning::CartesianConstraint::Ptr _manifold;
    Planning::ValidityCheckContext _vc_context;

    ros::Subscriber _goal_sub, _start_sub;
    XBot::ModelInterface::Ptr _start_model, _goal_model;
    XBot::Cartesian::Planning::RobotViz::Ptr _start_viz, _goal_viz;

    ros::Publisher _trj_pub;
    ros::Publisher _pc_pub;
    ros::Publisher _contact_pub;
    std::vector<ros::Publisher> _cartesian_trajectory_publishers;
    std::vector<std::string> _base_links, _distal_links;

    ros::ServiceServer _planner_srv;
    ros::ServiceServer _get_planning_scene_srv;
    ros::ServiceServer _apply_planning_scene_srv;

    GoalGenerator::Ptr _goal_generator;
    bool _use_goal_generator;
    ros::ServiceServer _service_goal_sampler;

    CartesianTrajectoryInterpolation::Ptr _interpolator;

    bool _plan_controls;

    bool _callbackDone;
    Eigen::MatrixXd _pointCloud;
    std::shared_ptr<XBot::Planning::PointCloudManager> _pc_manager;
    
    Planning::NSPG::Ptr _NSPG;
    
    Eigen::Vector3d getNormalAtPoint(Eigen::Vector3d p);
    //Eigen::Matrix3d generateRotationAroundAxis(EndEffector ee, Eigen::Vector3d axis);
    bool computeIKandCS(Stance sigmaSmall, Stance sigmaLarge, Configuration qNear, Configuration &qNew, bool adding);
    
    XBot::Cartesian::RosServerClass::Ptr _ros_server;
    


};

} }

#endif // PLANNER_EXECUTOR_H
