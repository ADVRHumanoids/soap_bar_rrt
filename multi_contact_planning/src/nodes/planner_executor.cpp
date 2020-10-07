#include "planner_executor.h"

#include "utils/parse_yaml_utils.h"
#include "validity_checker/validity_checker_factory.h"

#include <trajectory_msgs/JointTrajectory.h>

#include <RobotInterfaceROS/ConfigFromParam.h>

#include <cartesian_interface/utils/LoadConfig.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>

#include "multi_contact_planning/CartesianTrajectory.h"

using namespace XBot::Cartesian;

void PlannerExecutor::writeOnFileConfigs(std::vector<Configuration> qList, std::string fileName){
    std::string filePrefix = "/home/paolo/catkin_ws/external/src/soap_bar_rrt/multi_contact_planning/PlanningData/";
    std::string filePath = filePrefix + fileName + ".txt";
    static std::ofstream fileOut(filePath, std::ofstream::trunc);
    
    for(int i = 0; i < qList.size(); i++){
        Configuration q = qList.at(i);
        Eigen::VectorXd c(n_dof);
        c.segment(0,3) = q.getFBPosition();
        c.segment(3,3) = q.getFBOrientation();
        c.tail(n_dof-6) = q.getJointValues();
        fileOut << c.transpose() << std::endl;
    } 
}

void PlannerExecutor::readFromFileConfigs(std::vector<Configuration> &qList, std::string fileName){
    std::string filePrefix = "/home/paolo/catkin_ws/external/src/soap_bar_rrt/multi_contact_planning/PlanningData/";
    std::string filePath = filePrefix + fileName + ".txt";
    std::ifstream fileIn(filePath.c_str());
    std::string line;       
    
    while(getline(fileIn, line)){
        std::stringstream linestream(line);
        std::string value;
        Eigen::VectorXd c(n_dof);
        int index = 0;

        while(linestream >> value){
            c(index) = boost::lexical_cast<double>(value);
            index++;
        }

        Configuration q;
        q.setFBPosition(c.segment(0,3));
        q.setFBOrientation(c.segment(3,3));
        q.setJointValues(c.tail(n_dof-6));
        qList.push_back(q);
    }
}

void PlannerExecutor::writeOnFileStances(std::vector<Stance> sigmaList, std::string fileName){
    std::string filePrefix = "/home/paolo/catkin_ws/external/src/soap_bar_rrt/multi_contact_planning/PlanningData/";
    std::string filePath = filePrefix + fileName + ".txt";
    static std::ofstream fileOut(filePath, std::ofstream::trunc);
    
    for(int i = 0; i < sigmaList.size(); i++){
        Stance sigma = sigmaList.at(i);
        fileOut << sigma.getSize() << std::endl;
        for(int j = 0; j < sigma.getSize(); j++){
            Contact* c = sigma.getContact(j);
            EndEffector ee = c->getEndEffectorName();
            Eigen::Affine3d T = c->getPose();
            Eigen::Vector3d F = c->getForce();
            Eigen::Vector3d n = c->getNormal();

            fileOut << ee << std::endl;
            fileOut << T.translation().transpose() << std::endl;
            fileOut << F.transpose() << std::endl;
            fileOut << n.transpose() << std::endl;
        }
    } 
}

void PlannerExecutor::readFromFileStances(std::vector<Stance> &sigmaList, std::string fileName){
    std::string filePrefix = "/home/paolo/catkin_ws/external/src/soap_bar_rrt/multi_contact_planning/PlanningData/";
    std::string filePath = filePrefix + fileName + ".txt";
    std::ifstream fileIn(filePath.c_str());
    std::string line;       
    
    while(getline(fileIn, line)){

        double sigma_size = boost::lexical_cast<double>(line);
        Stance sigma;
        std::string value;
        int index;

        for(int i = 0; i < sigma_size; i++){
            getline(fileIn, line); // end effector name
            EndEffector ee = (EndEffector)boost::lexical_cast<int>(line);
            
            getline(fileIn, line); // pose
            Eigen::Vector3d pos;
            std::stringstream pose_stream(line);
            index = 0;
            while(pose_stream >> value){
                pos(index) = boost::lexical_cast<double>(value);
                index++;
            }
            Eigen::Affine3d T;
            T.translation() = pos;
            T.linear() = Eigen::Matrix3d::Identity(3,3);

            getline(fileIn, line); // force
            Eigen::Vector3d F;
            std::stringstream force_stream(line);
            index = 0;
            while(force_stream >> value){
                F(index) = boost::lexical_cast<double>(value);
                index++;
            }
                
            getline(fileIn, line); // normal
            Eigen::Vector3d n;
            std::stringstream normal_stream(line);
            index = 0;
            while(normal_stream >> value){
                n(index) = boost::lexical_cast<double>(value);
                index++;
            }
                
            Contact* c = new Contact(ee, T, F, n);
            sigma.addContact(c);            
        }    
        
        sigmaList.push_back(sigma);
    }
}
            
PlannerExecutor::PlannerExecutor():
    _nh("planner"),
    _nhpr("~"), 
    _n()
{
    
    init_load_config();
    init_load_model();
    init_load_planner();
    init_load_validity_checker();
    init_goal_generator();
    init_subscribe_start_goal();
    init_trj_publisiher();
    init_planner_srv();
    init_interpolator();

    index_config = -1;
    plan.clear();

    std::cout << "EVERYTHING INITIALIZED" << std::endl;
}

void PlannerExecutor::run()
{
    auto time = ros::Time::now();
    ros::spinOnce();

    if(_use_goal_generator)
        _goal_generator->update();

    publish_and_check_start_and_goal_models(time);        
}

void PlannerExecutor::init_load_model()
{
    auto cfg = XBot::ConfigOptionsFromParamServer();
    _model = XBot::ModelInterface::getModel(cfg);
    _start_model = XBot::ModelInterface::getModel(cfg);
    _goal_model = XBot::ModelInterface::getModel(cfg);

    Eigen::VectorXd qhome;
    _model->getRobotState("home", qhome);

    _model->setJointPosition(qhome);
    _model->update();

    _start_model->setJointPosition(qhome);
    _start_model->update();

    _goal_model->setJointPosition(qhome);
    _goal_model->update();

    std::string world_frame_link;
    if(_nhpr.hasParam("world_frame_link"))
    {
        _nhpr.getParam("world_frame_link", world_frame_link);
        Eigen::Affine3d T;
        if(_model->getPose(world_frame_link,T))
        {
            ROS_INFO("Setting planner world frame in %s", world_frame_link.c_str());

            _model->setFloatingBasePose(T.inverse());
            _model->update();

            _start_model->setFloatingBasePose(T.inverse());
            _start_model->update();

            _goal_model->setFloatingBasePose(T.inverse());
            _goal_model->update();
        }
        else
            ROS_ERROR("world_frame_link %s does not exists, keeping original world!", world_frame_link.c_str());
    }

    ////////////////////////////////////////////////////////////////////////////////////////// PF
    n_dof = _model->getJointNum();

    Eigen::VectorXd qInit(n_dof);

    qInit << 0.0883568, -0.126304, 0.639739, 1.10568, -4.72852, -1.10301, 0.0258766, -0.989014, 0.0479682, 0.0473017, -0.621278, -0.0289819, 0.0358694, -0.963558, 0.0608695, 0, -0.599092, -0.0373323, 0.0504259, 0.0425578, -2.16338, 1.03381, 1.70575, -0.611303, 2.34071, -0.972389, 0.272461, -0.322765, -2.36409, 0.584142, -1.21375, 0.540567, -0.155282, 1.9109;

    _model->setJointPosition(qInit);
    _model->update();

    _start_model->setJointPosition(qInit);
    _start_model->update();

    _goal_model->setJointPosition(qInit);
    _goal_model->update();
    //////////////////////////////////////////////////////////////////////////////////////////  
        
    //_pc_manager = std::make_shared<XBot::Planning::PointCloudManager>(_n, "filtered_cloud2");

    Eigen::Affine3d T;
    _model->getPose("l_sole", T);

    Eigen::Vector3d center;
    center << 0.0, 0.0, T.translation().z();
    double side_x = 2.0;
    double side_y = 2.0;
    double side_z = 3.0;
    double resolution = 0.1;

    _pc_manager = std::make_shared<XBot::Planning::PointCloudManager>(_n, center, side_x, side_y, side_z, resolution);
    _pc_manager->computeNormals(0.2);
    _pc_manager->fromNormaltoMarkerArray(0.2);

    _pc_pub = _n.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud", 1, true);
    _pc_pub.publish(_pc_manager->getPCLPointCloud());
}

void PlannerExecutor::init_load_config()
{
    if(!_nhpr.hasParam("planner_config"))
    {
        throw std::runtime_error("Mandatory private parameter 'planner_config' missing");
    }

    // load planner config file (yaml)
    std::string planner_config_string;
    _nhpr.getParam("planner_config", planner_config_string);

    _planner_config = YAML::Load(planner_config_string);
}

void PlannerExecutor::init_load_planner()
{
    // manifold variable
    ompl::base::ConstraintPtr ompl_constraint;

    // if a manifold was specified, load it from param server
    if(_planner_config["manifold"])
    {
        YAML_PARSE_OPTION(_planner_config["manifold"],
                param_name,
                std::string,
                "constraint_problem_description");




        std::string problem_description_string;
        YAML::Node ik_yaml_constraint;

        if(!_nh.hasParam(param_name) ||
                !_nh.getParam(param_name,
                              problem_description_string))
        {
            throw std::runtime_error("problem_description '" + param_name + "' parameter missing");
        }

        _manifold = make_manifold(problem_description_string);

        ompl_constraint = _manifold;

    }

    // get state bounds
    Eigen::VectorXd qmin, qmax;
    _model->getJointLimits(qmin, qmax);

    Eigen::VectorXd qdotlims; //[rad/sec]
    _model->getVelocityLimits(qdotlims);

    if(_planner_config["control_space"])
    {
        _plan_controls = true;
        ROS_INFO("Planner works in control space!");
    }
    else
    {
        _plan_controls = false;
        ROS_INFO("Planner works in state space!");
    }

    if(_model->isFloatingBase())
    {
        qmax.head<6>() << 1.0, 1.0, 1.0, 2*M_PI, 2*M_PI, 2*M_PI;
        qmin.head<6>() << -qmax.head<6>();

        YAML_PARSE_OPTION(_planner_config["state_space"],
                floating_base_pos_min,
                std::vector<double>,
                {});

        if(floating_base_pos_min.size() > 0)
        {
            qmin.head<3>() << floating_base_pos_min[0],
                    floating_base_pos_min[1],
                    floating_base_pos_min[2];
        }

        YAML_PARSE_OPTION(_planner_config["state_space"],
                floating_base_pos_max,
                std::vector<double>,
                {});

        if(floating_base_pos_max.size() > 0)
        {
            qmax.head<3>() << floating_base_pos_max[0],
                    floating_base_pos_max[1],
                    floating_base_pos_max[2];
        }

        if(_plan_controls)
        {
            qdotlims.head<6>() << 1., 1., 1., 2*M_PI, 2*M_PI, 2*M_PI;

            YAML_PARSE_OPTION(_planner_config["control_space"],
                    floating_base_velocity_limits,
                    std::vector<double>,
                    {});

            if(floating_base_velocity_limits.size() > 0)
            {
                for(unsigned int i = 0; i < floating_base_velocity_limits.size(); ++i)
                    qdotlims[i] = floating_base_velocity_limits[i];

            }

        }

    }



    if(ompl_constraint)
    {
        ROS_INFO("Constructing a constrained ompl planner");

        if(_plan_controls)
        {
            _planner = std::make_shared<Planning::OmplPlanner>(qmin, qmax, -qdotlims, qdotlims, ompl_constraint, _planner_config);
        }
        else
        {
            _planner = std::make_shared<Planning::OmplPlanner>(qmin, qmax, ompl_constraint, _planner_config);
        }


    }
    else
    {
        ROS_INFO("Constructing an unconstrained ompl planner");

        if(_plan_controls)
        {
            _planner = std::make_shared<Planning::OmplPlanner>(qmin, qmax, -qdotlims, qdotlims, _planner_config);
        }
        else
        {
            _planner = std::make_shared<Planning::OmplPlanner>(qmin, qmax, _planner_config);
        }
    }




}

void PlannerExecutor::init_load_validity_checker()
{
    _vc_context = Planning::ValidityCheckContext(_planner_config,
                                                 _model, _nh);

    _vc_context.planning_scene->startMonitor();

    _vc_context.planning_scene->startMonitor();

   
    ///////////////////////////////////////////////////////////////////////
    _vc_context.planning_scene->acm.setEntry("LBall", "<octomap>", true);
    _vc_context.planning_scene->acm.setEntry("LFoot", "<octomap>", true);
    //_vc_context.planning_scene->acm.setEntry("LFootmot", "<octomap>", true);
    //_vc_context.planning_scene->acm.setEntry("LLowLeg", "<octomap>", true);
    //_vc_context.planning_scene->acm.setEntry("LWrMot2", "<octomap>", true);
    //_vc_context.planning_scene->acm.setEntry("LWrMot3", "<octomap>", true);
    _vc_context.planning_scene->acm.setEntry("RBall", "<octomap>", true);
    _vc_context.planning_scene->acm.setEntry("RFoot", "<octomap>", true);
    //_vc_context.planning_scene->acm.setEntry("RFootmot", "<octomap>", true);
    //_vc_context.planning_scene->acm.setEntry("RLowLeg", "<octomap>", true);
    //_vc_context.planning_scene->acm.setEntry("RWrMot2", "<octomap>", true);
    //_vc_context.planning_scene->acm.setEntry("RWrMot3", "<octomap>", true);
    ///////////////////////////////////////////////////////////////////////
   

    _get_planning_scene_srv = _nh.advertiseService("get_planning_scene",
                                                   &PlannerExecutor::get_planning_scene_service, this);

    _apply_planning_scene_srv = _nh.advertiseService("apply_planning_scene",
                                                     &PlannerExecutor::apply_planning_scene_service, this);

    auto validity_predicate = [this](const Eigen::VectorXd& q)
    {
        _model->setJointPosition(q);
        _model->update();
        return _vc_context.vc_aggregate.checkAll();
    };

    _planner->setStateValidityPredicate(validity_predicate);
}

void PlannerExecutor::init_subscribe_start_goal()
{
    _start_sub = _nh.subscribe("start/joint_states", 1,
                               &PlannerExecutor::on_start_state_recv, this);

    if(!_use_goal_generator)
        _goal_sub = _nh.subscribe("goal/joint_states", 1,
                                  &PlannerExecutor::on_goal_state_recv, this);

    _start_viz = std::make_shared<Planning::RobotViz>(_model,
                                                      "start/robot_markers",
                                                      _nh);
    _start_viz->setPrefix("planner/start/");

    _goal_viz = std::make_shared<Planning::RobotViz>(_model,
                                                     "goal/robot_markers",
                                                     _nh);
    _goal_viz->setPrefix("planner/goal/");
}

void PlannerExecutor::init_trj_publisiher()
{
    if(_nhpr.hasParam("distal_links"))
    {
        XmlRpc::XmlRpcValue v;
        _nhpr.getParam("distal_links", v);
        for(int i =0; i < v.size(); i++)
            _distal_links.push_back(v[i]);

        if(_nhpr.hasParam("base_links"))
        {
            XmlRpc::XmlRpcValue v;
            _nhpr.getParam("base_links", v);
            for(int i =0; i < v.size(); i++)
                _base_links.push_back(v[i]);
        }
        else
        {
            for(unsigned int i = 0; i < _distal_links.size(); ++i)
                _base_links.push_back("world");
        }


        if(_base_links.size() != _distal_links.size())
        {
            throw std::runtime_error("base_links and distal_links params should have same size!");
        }

        for(unsigned int i = 0; i < _distal_links.size(); ++i)
            _cartesian_trajectory_publishers.push_back(_nh.advertise<multi_contact_planning::CartesianTrajectory>
                                                       (_distal_links[i]+"/cartesian_trajectory",1, true));
    }

    _trj_pub = _nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1, true);
}

void PlannerExecutor::init_planner_srv()
{
    _planner_srv = _nh.advertiseService("compute_plan", &PlannerExecutor::planner_service, this);
}

void PlannerExecutor::init_goal_generator()
{
    if(!_nhpr.getParam("use_goal_generator" ,_use_goal_generator))
        _use_goal_generator = false;

    if(_use_goal_generator)
    {
        std::string problem_description_string;
        if(!_nh.getParam("problem_description_goal", problem_description_string))
        {
            ROS_ERROR("planner/problem_description_goal not provided!");
            throw std::runtime_error("planner/problem_description_goal not provided!");
        }

        auto ik_yaml_goal = YAML::Load(problem_description_string);

        double ci_period = 1.0;
        auto ci_ctx = std::make_shared<Context>(
                    std::make_shared<Parameters>(ci_period),
                    _model);

        auto ik_prob = ProblemDescription(ik_yaml_goal, ci_ctx);

        auto ci = CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                       ik_prob, ci_ctx);

        /////////////////////////////////////////////////////////////////////////////
        //auto task_0 = ci->getTask("Com");

        /////////////////////////////////////////////////////////////////////////////

        _goal_generator = std::make_shared<GoalGenerator>(ci, _vc_context);

        int max_iterations;
        if(_nhpr.getParam("goal_generator_max_iterations", max_iterations))
        {
            _goal_generator->setMaxIterations(max_iterations);
        }

        double error_tolerance;
        if(_nhpr.getParam("goal_generator_error_tolerance", error_tolerance))
        {
            _goal_generator->setErrorTolerance(error_tolerance);
        }
        else
        {
            if(_manifold)
                _goal_generator->setErrorTolerance(_manifold->getTolerance());
        }

        _service_goal_sampler = _nh.advertiseService("goal_sampler_service",
                                                     &PlannerExecutor::goal_sampler_service, this);

        ROS_WARN("goal generator is going to be used, disabling goal from topic");
    }
}

void PlannerExecutor::init_interpolator()
{
    _interpolator = std::make_shared<CartesianTrajectoryInterpolation>();

    ///TODO: qdot, qddot limits? 
}

/* 
void PlannerExecutor::setReferences(std::vector<std::string> active_tasks, std::vector<Eigen::Affine3d> ref_tasks, Eigen::VectorXd q_ref){
    std::cout << "SETTING REFERENCES" << std::endl;

    auto ci = _goal_generator->getCartesianInterface(); 

    std::vector<std::string> active_tasks_all;
    active_tasks_all.push_back("Com");
    active_tasks_all.push_back("TCP_L");
    active_tasks_all.push_back("TCP_R");
    active_tasks_all.push_back("l_sole");
    active_tasks_all.push_back("r_sole");

    for(int i = 0; i < active_tasks_all.size(); i++){
        ci->setActivationState(active_tasks_all.at(i), ActivationState::Disabled);
        //ci->getTask(active_tasks_all.at(i))->setLambda(0.1);   
    }
   
    Eigen::Affine3d T_ref;
    for(int i = 0; i < active_tasks.size(); i++){
        ci->setActivationState(active_tasks.at(i), ActivationState::Enabled);
        //ci->getTask(active_tasks.at(i))->setLambda(1.0);   
        T_ref = ref_tasks.at(i);
        ci->setPoseReference(active_tasks.at(i), T_ref);
    }
  
    XBot::JointNameMap jmap;
    _goal_model->eigenToMap(q_ref, jmap);
    //ci->setReferencePosture(jmap);

    for (auto i : active_tasks)
    {
        if (i == "Com") continue;
        else if (i == "TCP_R") _vc_context.planning_scene->acm.setEntry("RBall", "<octomap>", true);   
        else if (i == "TCP_L") _vc_context.planning_scene->acm.setEntry("LBall", "<octomap>", true);     
        else if (i == "l_sole") _vc_context.planning_scene->acm.setEntry("LFoot", "<octomap>", true);    
        else if (i == "r_sole") _vc_context.planning_scene->acm.setEntry("RFoot", "<octomap>", true);    
    }
}
*/
 
/* 
void PlannerExecutor::setReferences(std::vector<std::string> active_tasks, std::vector<Eigen::Affine3d> ref_tasks, Eigen::VectorXd q_ref){
    std::cout << "SETTING REFERENCES" << std::endl;

    auto ci = _goal_generator->getCartesianInterface();

    std::vector<std::string> active_tasks_all;
    active_tasks_all.push_back("Com");
    active_tasks_all.push_back("TCP_L");
    active_tasks_all.push_back("TCP_R");
    active_tasks_all.push_back("l_sole");
    active_tasks_all.push_back("r_sole");

    for(int i = 0; i < active_tasks_all.size(); i++){
        int index = -1;
        for(int j = 0; j < active_tasks.size(); j++) if(active_tasks[j] == active_tasks_all[i]) index = j;
    
        if(index == -1) ci->setActivationState(active_tasks_all[i], ActivationState::Disabled);
        else{
            ci->setActivationState(active_tasks_all[i], ActivationState::Enabled);
            ci->setPoseReference(active_tasks_all[i], ref_tasks[index]);
        }
    }   
    
    
    //_vc_context.planning_scene->acm.clear(); // this does not work
    _vc_context.planning_scene->acm.setEntry("RBall", "<octomap>", false);   
    _vc_context.planning_scene->acm.setEntry("LBall", "<octomap>", false);     
    _vc_context.planning_scene->acm.setEntry("LFoot", "<octomap>", false);    
    _vc_context.planning_scene->acm.setEntry("RFoot", "<octomap>", false);    
    for (auto i : active_tasks) {
        if (i == "Com") continue;
        else if (i == "TCP_R") _vc_context.planning_scene->acm.setEntry("RBall", "<octomap>", true);   
        else if (i == "TCP_L") _vc_context.planning_scene->acm.setEntry("LBall", "<octomap>", true);     
        else if (i == "l_sole") _vc_context.planning_scene->acm.setEntry("LFoot", "<octomap>", true);    
        else if (i == "r_sole") _vc_context.planning_scene->acm.setEntry("RFoot", "<octomap>", true);    
    } 
   

    XBot::JointNameMap jmap;
    _goal_model->eigenToMap(q_ref, jmap);
    ci->setReferencePosture(jmap);
}
*/

void PlannerExecutor::setReferences(std::vector<std::string> active_tasks, std::vector<Eigen::Affine3d> ref_tasks, Eigen::VectorXd q_ref){
    std::cout << "SETTING REFERENCES" << std::endl;

    auto ci = _goal_generator->getCartesianInterface();

    std::vector<std::string> active_tasks_all;
    active_tasks_all.push_back("Com");
    active_tasks_all.push_back("TCP_L");
    active_tasks_all.push_back("TCP_R");
    active_tasks_all.push_back("l_sole");
    active_tasks_all.push_back("r_sole");

    for(int i = 0; i < active_tasks_all.size(); i++){
        ci->setPoseReference(active_tasks_all[i], ref_tasks[i]);
    }

    /*
    for(int i = 0; i < active_tasks_all.size(); i++){
        int index = -1;
        for(int j = 0; j < active_tasks.size(); j++) if(active_tasks[j] == active_tasks_all[i]) index = j;
    
        //if(index == -1) ci->getTask(active_tasks_all.at(i))->setWeight(0.01*Eigen::MatrixXd::Identity(ci->getTask(active_tasks_all.at(i))->getWeight().rows(), ci->getTask(active_tasks_all.at(i))->getWeight().cols()));  
        if(index == -1) ci->setActivationState(active_tasks_all[i], ActivationState::Disabled);
        else{
            //ci->getTask(active_tasks_all.at(i))->setWeight(Eigen::MatrixXd::Identity(ci->getTask(active_tasks_all.at(i))->getWeight().rows(), ci->getTask(active_tasks_all.at(i))->getWeight().cols()));
            ci->setActivationState(active_tasks_all[i], ActivationState::Enabled);
            ci->setPoseReference(active_tasks_all[i], ref_tasks[index]);
        }
    }   
    */

    /*
    //_vc_context.planning_scene->acm.clear(); // this does not work
    _vc_context.planning_scene->acm.setEntry("RBall", "<octomap>", false);   
    _vc_context.planning_scene->acm.setEntry("LBall", "<octomap>", false);     
    _vc_context.planning_scene->acm.setEntry("LFoot", "<octomap>", false);    
    _vc_context.planning_scene->acm.setEntry("RFoot", "<octomap>", false);    
    for (auto i : active_tasks) {
        if (i == "Com") continue;
        else if (i == "TCP_R") _vc_context.planning_scene->acm.setEntry("RBall", "<octomap>", true);   
        else if (i == "TCP_L") _vc_context.planning_scene->acm.setEntry("LBall", "<octomap>", true);     
        else if (i == "l_sole") _vc_context.planning_scene->acm.setEntry("LFoot", "<octomap>", true);    
        else if (i == "r_sole") _vc_context.planning_scene->acm.setEntry("RFoot", "<octomap>", true);    
    } 
    */

    XBot::JointNameMap jmap;
    _goal_model->eigenToMap(q_ref, jmap);
    //ci->setReferencePosture(jmap);
}


bool PlannerExecutor::goal_sampler_service(multi_contact_planning::CartesioGoal::Request &req,
                                           multi_contact_planning::CartesioGoal::Response &res)
{

    std::cout << "CALL TO THE GOAL SAMPLER" << std::endl; 

     
    /////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<std::string> active_tasks;
    std::vector<Eigen::Affine3d> ref_tasks;
    Eigen::VectorXd q_ref;

    active_tasks.push_back("Com");
    active_tasks.push_back("TCP_L");
    active_tasks.push_back("TCP_R");
    active_tasks.push_back("l_sole");
    active_tasks.push_back("r_sole");

    _goal_model->getJointPosition(q_ref); // for postural task 

    Eigen::Affine3d T_ref;
    Eigen::Matrix3d rot_ref = Eigen::Matrix3d::Identity(3,3);
    Eigen::Vector3d pos_ref;

    //COM 
    pos_ref << 0.113322, -0.312488, 0.484222;
    T_ref.translation() = pos_ref;
    T_ref.linear() = rot_ref;
    ref_tasks.push_back(T_ref);
    //LH 
    pos_ref << 0.7, 0.4, 0.0; //1.0, 0.2, 0.5;//
    T_ref.translation() = pos_ref;
    T_ref.linear() = rot_ref;
    ref_tasks.push_back(T_ref);
    //RH
    pos_ref << 0.7, -0.6, 0.0;
    T_ref.translation() = pos_ref;
    T_ref.linear() = rot_ref;
    ref_tasks.push_back(T_ref);
    //LF
    pos_ref << -0.5, 0.0, 0.0;  
    T_ref.translation() = pos_ref;
    T_ref.linear() = rot_ref;
    ref_tasks.push_back(T_ref);
    //RF
    pos_ref << -0.5, -0.2, 0.0; 
    T_ref.translation() = pos_ref;
    T_ref.linear() = rot_ref;
    ref_tasks.push_back(T_ref);
    

    setReferences( active_tasks, ref_tasks, q_ref );
     
  
    Eigen::VectorXd q;
    if(!_goal_generator->sample(q, req.time)){
        res.status.val = res.status.TIMEOUT;
        res.status.msg.data = "TIMEOUT";
    }
    else
    {
        res.status.val = res.status.EXACT_SOLUTION;
        res.status.msg.data = "EXACT_SOLUTION";

        res.sampled_goal.name = _goal_model->getEnabledJointNames();
        res.sampled_goal.position.resize(q.size());
        Eigen::VectorXd::Map(&res.sampled_goal.position[0], q.size()) = q;
        res.sampled_goal.header.stamp = ros::Time::now();
    }
    
    /*
    if(_manifold) 
        _manifold->project(q);  // what s this???
    */    
    
    _goal_model->setJointPosition(q);
    _goal_model->update();

    for(int z = 0; z < q.rows(); z++) std::cout << q(z) << ", ";
    std::cout << " " << std::endl;

    return true;
}

Planning::CartesianConstraint::Ptr PlannerExecutor::make_manifold(std::string problem_description_string)
{

    auto ik_yaml_constraint = YAML::Load(problem_description_string);

    double ci_period = 1.0;
    auto ci_ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(ci_period),
                _model);

    auto ik_prob_constraint = ProblemDescription(ik_yaml_constraint, ci_ctx);

    auto constraint_ci = CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                              ik_prob_constraint,
                                                              ci_ctx);


    auto ik_solver = std::make_shared<Planning::PositionCartesianSolver>(constraint_ci);

    return std::make_shared<Planning::CartesianConstraint>(ik_solver);
}

bool PlannerExecutor::check_state_valid(XBot::ModelInterface::ConstPtr model)
{
    if(_model != model)
    {
        _model->syncFrom(*model, XBot::Sync::Position);
    }

    bool valid = true;

    std::vector<std::string> failed_checks;
    if(!_vc_context.vc_aggregate.checkAll(&failed_checks))
    {
        valid = false;

        std::cout << "Invalid state, failed validity checks were: \n";
        for(int i = 0; i < failed_checks.size(); i++)
        {
            std::cout << " - '" << failed_checks[i] << "'\n";
        }
        std::cout.flush();
    }

    Eigen::VectorXd q;
    Eigen::VectorXd qmin, qmax;
    _model->getJointPosition(q);
    _planner->getBounds(qmin, qmax);
    const double q_tol = 1e-3;

    if((q.array() < qmin.array() - q_tol).any() || (q.array() > qmax.array() + q_tol).any())
    {
        valid = false;
        std::cout << "Invalid state, violates bounds" << std::endl;

        for(int i = 0; i < _model->getJointNum(); i++)
        {
            if(q[i] < qmin[i] || q[i] > qmax[i])
            {
                std::cout << _model->getEnabledJointNames().at(i) <<
                             ": " << qmin[i] << " <= " << q[i] <<
                             " <= " << qmax[i] << "\n";
            }
        }
        std::cout.flush();
    }

    if(!_manifold)
    {
        return valid;
    }

    Eigen::VectorXd error(_manifold->getCoDimension());
    _manifold->function(q, error);
    _manifold->getTolerance();


    const double err_threshold = _manifold->getTolerance();
    double err = error.cwiseAbs().maxCoeff();
    if(err > err_threshold)
    {
        valid = false;
        std::cout << "Invalid state, not on manifold (error = " << err << " > "<<err_threshold<<")" << std::endl;
    }

    return valid;
}

void PlannerExecutor::setStartState(const XBot::JointNameMap& q)
{
    _start_model->setJointPosition(q);
    _start_model->update();

    if(_manifold)
    {
        if(_model != _start_model)
        {
            _model->syncFrom(*_start_model, XBot::Sync::Position);
        }
        _manifold->reset(); // note: manifold is set according to start state
    }
}

void PlannerExecutor::on_start_state_recv(const sensor_msgs::JointStateConstPtr & msg)
{
    // tbd: input checking

    XBot::JointNameMap q;
    for(int i = 0; i < msg->name.size(); i++)
    {
        q[msg->name[i]] = msg->position[i];
    }


    setStartState(q);

}

void PlannerExecutor::setGoalState(const XBot::JointNameMap& q)
{
    _goal_model->setJointPosition(q);
    _goal_model->update();

    Eigen::VectorXd qq;
    _goal_model->getJointPosition(qq);

    if(_manifold)
        _manifold->project(qq);

    _goal_model->setJointPosition(qq);
    _goal_model->update();
}

void PlannerExecutor::on_goal_state_recv(const sensor_msgs::JointStateConstPtr & msg)
{
    // tbd: input checking

    XBot::JointNameMap q;
    for(int i = 0; i < msg->name.size(); i++)
    {
        q[msg->name[i]] = msg->position[i];
    }

    setGoalState(q);
}

bool PlannerExecutor::planner_service(multi_contact_planning::CartesioPlanner::Request& req,
                                      multi_contact_planning::CartesioPlanner::Response& res)
{
    
    std::cout << "+++++++++++++ PLANNING +++++++++++++" << std::endl;
        
    auto ci = _goal_generator->getCartesianInterface();
        
    // retrieve start

    Configuration qInit;
    Eigen::VectorXd qstart;
    _start_model->getJointPosition(qstart);
    Eigen::Vector3d pFB(qstart(0), qstart(1), qstart(2));
    Eigen::Vector3d eFB(qstart(3), qstart(4), qstart(5));
    Eigen::VectorXd jnt = qstart.tail(n_dof-6);
    qInit.setFBPosition(pFB);
    qInit.setFBOrientation(eFB);
    qInit.setJointValues(jnt);  

    std::vector<EndEffector> activeEEsInit;
    std::vector<Eigen::Affine3d> poseActiveEEsInit;
    Eigen::Affine3d T_init;
        
    activeEEsInit.clear();
    poseActiveEEsInit.clear();
    activeEEsInit.push_back(L_HAND);
    activeEEsInit.push_back(R_HAND); 
    activeEEsInit.push_back(L_FOOT);
    activeEEsInit.push_back(R_FOOT);
    ci->getCurrentPose("TCP_L", T_init);
    poseActiveEEsInit.push_back(T_init);
    ci->getCurrentPose("TCP_R", T_init);
    poseActiveEEsInit.push_back(T_init);
    ci->getCurrentPose("l_sole", T_init);
    poseActiveEEsInit.push_back(T_init);
    ci->getCurrentPose("r_sole", T_init);
    poseActiveEEsInit.push_back(T_init);
        
    // construct goal 

    std::vector<EndEffector> activeEEsGoal;
    std::vector<Eigen::Affine3d> poseActiveEEsGoal;
    Eigen::Affine3d T_goal;
    Eigen::Matrix3d rot_goal = Eigen::Matrix3d::Identity(3,3);
    Eigen::Vector3d pos_goal;
        
    activeEEsGoal.clear();
    poseActiveEEsGoal.clear();
    activeEEsGoal.push_back(L_HAND);
    activeEEsGoal.push_back(R_HAND);
    activeEEsGoal.push_back(L_FOOT);
    activeEEsGoal.push_back(R_FOOT); 

    //LH
    pos_goal << 1.0, 0.2, 1.4;
    //pos_goal << 0.8, 0.4, 0.0; // init 0.7   
    T_goal.translation() = pos_goal;
    T_goal.linear() = rot_goal;
    poseActiveEEsGoal.push_back(T_goal);
    //RH
    pos_goal << 1.0, -0.4, 1.4;
    //pos_goal << 0.7, -0.6, 0.0; // init 
    T_goal.translation() = pos_goal;
    T_goal.linear() = rot_goal;
    poseActiveEEsGoal.push_back(T_goal);
    //LF
    pos_goal << 0.0, 0.0, 0.0;
    //pos_goal << -0.2, 0.1, 0.0;
    //pos_goal << -0.5, 0.0, 0.0; // init
    T_goal.translation() = pos_goal;
    T_goal.linear() = rot_goal;
    poseActiveEEsGoal.push_back(T_goal);
    //RF
    pos_goal << 0.0, -0.2, 0.0;
    //pos_goal << -0.5, -0.2, 0.0; // init
    T_goal.translation() = pos_goal;
    T_goal.linear() = rot_goal;
    poseActiveEEsGoal.push_back(T_goal);
            
    // construct the environment description    

    Eigen::MatrixXd pointCloud = _pc_manager->getPointCloud();
    Eigen::MatrixXd pointNormals = _pc_manager->getNormals();

    // construct allowed end-effectors description
    
    std::vector<EndEffector> allowedEEs;
    allowedEEs.clear();
    allowedEEs.push_back(L_FOOT);
    allowedEEs.push_back(R_FOOT);
    allowedEEs.push_back(L_HAND);
    allowedEEs.push_back(R_HAND);  

    // plan a solution    
       
    std::vector<Stance> sigmaList_1, sigmaList_2;
    std::vector<Configuration> qList_1, qList_2;
    bool sol_found_1, sol_found_2;

    bool stage_1 = false; 
    bool stage_2 = false;

    if(index_config == -1){
        
        // create/initialize the planner
        Planner* planner = new Planner(qInit, poseActiveEEsInit, activeEEsInit, poseActiveEEsGoal, activeEEsGoal, pointCloud, pointNormals, allowedEEs, _model, _goal_generator, _vc_context);
        std::cout << "planner created!" << std::endl;

        if(stage_1){
            // run 1st stage
            float t_elapsed_1 = 0.0;
            auto t_start_chrono_1 = std::chrono::steady_clock::now();
            planner->run1stStage();
            std::cout << "1st stage completed!" << std::endl;
            auto t_curr_chrono_1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t_start_chrono_1).count();
            t_elapsed_1 = (float)t_curr_chrono_1 / 1000.0;
            std::cout << "Planning Time 1st Stage:: " << t_elapsed_1 << std::endl;  
                
            // retrieve 1st stage solution
            sigmaList_1.clear();
            qList_1.clear();
            sol_found_1 = planner->retrieveSolution1stStage(sigmaList_1, qList_1);
            if(sol_found_1) std::cout << "1st Stage Solution FOUND!" << std::endl;
            else std::cout << "1st Stage Solution NOT FOUND!" << std::endl;
            std::cout << "sigmaList_1.size() = " << sigmaList_1.size() << std::endl;
            std::cout << "qList_1.size() = " << qList_1.size() << std::endl;
            std::cout << "tree.size() = " << planner->getTreeSize() << std::endl;
            writeOnFileConfigs(qList_1, "qList_1");
            writeOnFileStances(sigmaList_1, "sigmaList_1");
        }
        else{
            sigmaList_1.clear();
            qList_1.clear();    
            readFromFileConfigs(qList_1, "qList_1");
            readFromFileStances(sigmaList_1, "sigmaList_1");
            sol_found_1 = true;      
        }
        
        if(sol_found_1){

            if(stage_2){
                // run 2nd stage
                float t_elapsed_2 = 0.0;
                auto t_start_chrono_2 = std::chrono::steady_clock::now();
                planner->run2ndStage(sigmaList_1, qList_1);
                std::cout << "2nd stage completed!" << std::endl;
                auto t_curr_chrono_2 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t_start_chrono_2).count();
                t_elapsed_2 = (float)t_curr_chrono_2 / 1000.0;
                std::cout << "Planning Time 2nd Stage:: " << t_elapsed_2 << std::endl;  
                
                // retrieve 2nd stage solution
                sigmaList_2.clear();
                qList_2.clear();
                sol_found_2 = planner->retrieveSolution2ndStage(sigmaList_2, qList_2);
                if(sol_found_2) std::cout << "2nd Stage Solution FOUND!" << std::endl;
                else std::cout << "2nd Stage Solution NOT FOUND!" << std::endl;
                std::cout << "sigmaList_2.size() = " << sigmaList_2.size() << std::endl;
                std::cout << "qList_2.size() = " << qList_2.size() << std::endl;
                writeOnFileConfigs(qList_2, "qList_2");
                writeOnFileStances(sigmaList_2, "sigmaList_2");   
            }
            else{
                sigmaList_2.clear();
                qList_2.clear();    
                readFromFileConfigs(qList_2, "qList_2");
                readFromFileStances(sigmaList_2, "sigmaList_2");
                sol_found_2 = true;            
            }

        }

        _vc_context.planning_scene->acm.setEntry("RBall", "<octomap>", true);   
        _vc_context.planning_scene->acm.setEntry("LBall", "<octomap>", true);     
        _vc_context.planning_scene->acm.setEntry("LFoot", "<octomap>", true);    
        _vc_context.planning_scene->acm.setEntry("RFoot", "<octomap>", true);    
    
    } 

    
    // this is for DEBUGGING
    if(index_config == -1 && sol_found_2){
        for(int i = 0; i < qList_2.size(); i++){
            Configuration q = qList_2.at(i);
            Eigen::VectorXd c(n_dof);
            c.segment(0,3) = q.getFBPosition();
            c.segment(3,3) = q.getFBOrientation();
            c.tail(n_dof-6) = q.getJointValues();
            plan.push_back(c);
        }    
        index_config++; 
    }

    if(plan.size() > 0){
        _goal_model->setJointPosition(plan[index_config]);
        _goal_model->update();    
          
        index_config++;
                
        std::cout << "index_config = " << index_config << std::endl;
        std::cout << "plan.size() = " << plan.size() << std::endl;

        if(index_config == plan.size()) index_config = 0; 
    }
         
    
    
    return true; // if solution found
         
}



int PlannerExecutor::callPlanner(const double time, const std::string& planner_type, const double interpolation_time,
                                 const double goal_thrs,
                const std::vector<std::pair<std::string, std::string> > base_distal_links,
                std::vector<Eigen::VectorXd>& trajectory,
                std::vector<std::vector<Eigen::Affine3d> >& cartesian_trajectories)
{
    int ret = callPlanner(time, planner_type, goal_thrs, interpolation_time, trajectory);

    if(_interpolator->isValid())
    {
        for(unsigned int i = 0; i < base_distal_links.size(); ++i)
        {
            std::vector<Eigen::Affine3d> cartesian_trajectory;
            double t = 0.;
            while(t <= _interpolator->getTrajectoryEndTime())
            {
                cartesian_trajectory.push_back(_interpolator->evaluate(t, base_distal_links[i].first, base_distal_links[i].second));
                t += interpolation_time;
            }
            cartesian_trajectories.push_back(cartesian_trajectory);
        }

    }

    return ret;
}

int PlannerExecutor::callPlanner(const double time, const std::string& planner_type, const double goal_thrs,
                                 const double interpolation_time, std::vector<Eigen::VectorXd>& trajectory)
{
    if(time <= 0.)
        return ompl::base::PlannerStatus::ABORT;

    // check start and goal state correctness
    ROS_INFO("Checking start state validity..");
    if(!check_state_valid(_start_model))
    {
        throw std::runtime_error("Invalid start state");
    }

    ROS_INFO("Checking goal state validity..");
    if(!check_state_valid(_goal_model))
    {
        throw std::runtime_error("Invalid goal state");
    }

    ROS_INFO("start and goal states are valid");

    Eigen::VectorXd qstart, qgoal;
    _start_model->getJointPosition(qstart);
    _goal_model->getJointPosition(qgoal);

    ROS_INFO("Enforcing bounds...");
    enforce_bounds(qstart);
    enforce_bounds(qgoal);
    std::cout<<"...done!"<<std::endl;


    _planner->setStartAndGoalStates(qstart, qgoal, goal_thrs);

    _planner->solve(time, planner_type);


    std::vector<Eigen::VectorXd> raw_trajectory;
    if(_planner->getPlannerStatus())
        raw_trajectory = _planner->getSolutionPath();

    _interpolator->compute(raw_trajectory);
    double t = 0.;
    while(t <= _interpolator->getTrajectoryEndTime())
    {
        trajectory.push_back(_interpolator->evaluate(t));
        t += interpolation_time;
    }

    return ompl::base::PlannerStatus::StatusType(_planner->getPlannerStatus());
}

bool PlannerExecutor::get_planning_scene_service(moveit_msgs::GetPlanningScene::Request& req,
                                                 moveit_msgs::GetPlanningScene::Response& res)
{
    _vc_context.planning_scene->getPlanningScene(req, res);
    return true;
}

bool PlannerExecutor::apply_planning_scene_service(moveit_msgs::ApplyPlanningScene::Request & req,
                                                   moveit_msgs::ApplyPlanningScene::Response & res)
{
    _vc_context.planning_scene->applyPlanningScene(req.scene);
    return true;
}

void PlannerExecutor::publish_and_check_start_and_goal_models(ros::Time time)
{
    /* Publish start markers */
    auto start_color = (Eigen::Vector4d() << 0.0, 0.0, 1.0, 0.5).finished();

    bool start_valid = check_state_valid(_start_model);

    if(!start_valid)
    {
        start_color << 178./255., 0, 77./255., 0.5;
        ROS_WARN("START state is NOT valid!");
    }

    std::vector<std::string> red_links = _vc_context.planning_scene->getCollidingLinks();

    //for(unsigned int i = 0; i < red_links.size(); ++i)
        //ROS_WARN("start robot: colliding link %i --> %s",i ,red_links[i].c_str());

    _start_viz->setRGBA(start_color);
    _start_viz->publishMarkers(time, red_links);

    /* Publish goal markers */
    auto goal_color = (Eigen::Vector4d() << 0.0, 1.0, 0.0, 0.5).finished();

    bool goal_valid = check_state_valid(_goal_model);

    if(!goal_valid)
    {
        goal_color << 178./255., 77./255., 0, 0.5;
        ROS_WARN("GOAL state is NOT valid!");
    }

    red_links = _vc_context.planning_scene->getCollidingLinks();

    //for(unsigned int i = 0; i < red_links.size(); ++i)
        //ROS_WARN("goal robot: colliding link %i --> %s",i ,red_links[i].c_str());

    _goal_viz->setRGBA(goal_color);
    _goal_viz->publishMarkers(time, red_links);

}

void PlannerExecutor::enforce_bounds(Eigen::VectorXd & q) const
{
    Eigen::VectorXd qmin, qmax;
    _planner->getBounds(qmin, qmax);

    q = q.cwiseMin(qmax).cwiseMax(qmin);
}
