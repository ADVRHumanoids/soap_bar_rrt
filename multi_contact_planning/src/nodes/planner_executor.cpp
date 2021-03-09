#include "planner_executor.h"

#include "utils/parse_yaml_utils.h"
#include "validity_checker/validity_checker_factory.h"

#include <trajectory_msgs/JointTrajectory.h>

#include <RobotInterfaceROS/ConfigFromParam.h>

#include <cartesian_interface/utils/LoadConfig.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <cstdlib>

#include "multi_contact_planning/CartesianTrajectory.h"

#include "planner/multi_contact/utils.hpp"

using namespace XBot::Cartesian;

std::string env(std::getenv("ROBOTOLOGY_ROOT"));

void PlannerExecutor::writeOnFileConfigs(std::vector<Configuration> qList, std::string fileName){
    std::string filePrefix = env + "/soap_bar_rrt/multi_contact_planning/PlanningData/";
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
    std::string filePrefix = env + "/soap_bar_rrt/multi_contact_planning/PlanningData/";
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
    std::string filePrefix = env + "/soap_bar_rrt/multi_contact_planning/PlanningData/";
    std::string filePath = filePrefix + fileName + ".txt";
    static std::ofstream fileOut(filePath, std::ofstream::trunc);

    for(int i = 0; i < sigmaList.size(); i++){
        Stance sigma = sigmaList.at(i);
        fileOut << sigma.getSize() << std::endl;
        for(int j = 0; j < sigma.getSize(); j++){
            std::shared_ptr<Contact> c = sigma.getContact(j);
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
    std::string filePrefix = env + "/soap_bar_rrt/multi_contact_planning/PlanningData/";
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

            //Contact* c = new Contact(ee, T, F, n);
            std::shared_ptr<Contact> c = std::make_shared<Contact>(ee, T, F, n);
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

    q_init.resize(n_dof);
    q_goal.resize(n_dof);
    
    Eigen::VectorXd q0(n_dof); 
    Eigen::VectorXd q1(n_dof); 
    Eigen::VectorXd q2(n_dof); 
    Eigen::VectorXd q3(n_dof); 
    
    if(SCENARIO == 1){
        // STAND UP
        
        q0 << 0.392671, -0.0191674, 0.95783, -0.0105976, -0.0353956, 0.0267116, 0.0788019, -0.458175, -0.0334518, 0.741072, -0.24991, -0.0829859, -0.0311674, -0.446869, -0.0289934, 0.740588, -0.25764, 0.0292569, 0.00010873, -0.00163887, 0.956914, 0.00772743, 0.00150577, -1.91999, -0.000490356, -0.524224, -0.00193652, 0.960553, -0.00986163, 6.40194e-05, -1.91815, 0.000557603, -0.523711, 0.000680927;
        
        q1 << 0.759541, 0.0925172, 0.730835, 2.71415, 1.64804, -2.78167, 0.0077494, -1.71118, -0.0198973, 0.829911, -0.617268, 0.0637515, -0.550414, -1.50116, -0.576022, 0.514271, -0.500356, 0.104535, 0.425875, 0.077989, -0.94867, 0.0520036, 0.872267, -0.907853, -0.267958, 0.206271, 0.0805234, -0.933177, -0.2558, -0.273087, -0.964502, 0.679464, 0.561918, -0.236882;
        
        q2 << 1.58871, 0.0223469, 0.637945, 2.81551, 1.77831, -2.76061, 0.452002, -1.11945, 0.376866, 0.64416, -0.872665, -0.226634, -0.482266, -0.975, -0.58049, 0.55925, -0.872665, 0.256492, 0.0890082, -0.0296354, -1.01587, 1.07963, 0.14952, -0.864876, 0.690062, 0.809135, -0.146463, -0.943517, -1.21241, -0.186094, -0.878702, -0.766646, 0.841216, 0.190342;
            
        q3 << 1.76207, -0.0338754, 0.908798, 2.94557, -3.00842, 3.31202, 0.365677, -0.184716, -0.212292, 0.979485, -0.734834, -0.203572, 0.165889, 0.178129, -0.189333, 0.490139, -0.5617, 0.0652543, -0.0335189, -0.210854, -1.16491, 0.866977, -0.115734, -0.74658, 0.254145, 0.622937, 0.0555549, -1.26494, -0.840133, -1.00947, 0.0352932, -0.941131, 0.210595, 1.42731;
    }
    
    if(SCENARIO == 2){
        // PARALLEL WALLS CLIMBING
    
        q0 << 0.392671, -0.0191674, 0.95783, -0.0105976, -0.0353956, 0.0267116, 0.0788019, -0.458175, -0.0334518, 0.741072, -0.24991, -0.0829859, -0.0311674, -0.446869, -0.0289934, 0.740588, -0.25764, 0.0292569, 0.00010873, -0.00163887, 0.956914, 0.00772743, 0.00150577, -1.91999, -0.000490356, -0.524224, -0.00193652, 0.960553, -0.00986163, 6.40194e-05, -1.91815, 0.000557603, -0.523711, 0.000680927;
        
        q1 << 0.251611, -0.000849811, 1.04237, -0.0151705, -0.371729, -0.00205272, 0.911728, 0.0155281, 1.11786, 0.188712, -0.816631, 0.261799, -0.868397, 0.028154, -1.12254, 0.167069, -0.811616, -0.261799, 0.0637739, -0.128823, 1.45446, 2.19746, -0.788976, -1.03031, -1.56239, -0.582533, -0.95101, 1.20067, -2.54165, 0.738825, -1.05633, 1.58633, -0.797895, 1.14051;
        
        q2 << 0.251611, -0.000849811, 1.54237, -0.0151705, -0.371729, -0.00205272, 0.911728, 0.0155281, 1.11786, 0.188712, -0.816631, 0.261799, -0.868397, 0.028154, -1.12254, 0.167069, -0.811616, -0.261799, 0.0637739, -0.128823, 1.45446, 2.19746, -0.788976, -1.03031, -1.56239, -0.582533, -0.95101, 1.20067, -2.54165, 0.738825, -1.05633, 1.58633, -0.797895, 1.14051;
    }
    
    if(SCENARIO == 3){
        // LADDER CLIMBING
        
        q0 << 0.892671, -0.0191674, 0.95783, -0.0105976, -0.0353956, 0.0267116, 0.0788019, -0.458175, -0.0334518, 0.741072, -0.24991, -0.0829859, -0.0311674, -0.446869, -0.0289934, 0.740588, -0.25764, 0.0292569, 0.00010873, -0.00163887, 0.956914, 0.00772743, 0.00150577, -1.91999, -0.000490356, -0.524224, -0.00193652, 0.960553, -0.00986163, 6.40194e-05, -1.91815, 0.000557603, -0.523711, 0.000680927;

        q1 << 0.970538, 0.0335268, 0.975977, 0.0577962, -0.0604998, -0.029675, -0.0439653, -0.107748, 0.0327907, 0.385728, -0.218838, -0.0102487, -0.149492, -0.235458, 0.0400447, 0.588091, -0.297352, 0.101126, 0.00221161, 0.0111745, 0.0135683, 0.431521, 0.51865, -1.93329, 0.0651701, 0.6258, 0.384541, 0.018969, -0.611755, -0.512145, -1.91447, -0.17572, 0.670479, -0.333533;
        
        q2 << 1.35548, -0.00122826, 1.78233, -0.000595782, 0.017421, 0.0301185, 0.0548577, -0.117392, -0.0293313, 0.18532, -0.0869682, -0.0576443, -0.0549022, -0.155794, -0.0314119, 0.273305, -0.133266, 0.0505724, -0.000828284, -0.0245487, 0.00309155, 0.537416, 0.538394, -1.98381, 0.118169, 0.655286, 0.36424, 0.00824191, -0.523752, -0.547279, -1.99699, -0.108621, 0.662396, -0.364016;

        q3 << 1.9583, -0.000838725, 2.74617, -0.014519, -0.21955, 0.0121587, 0.0633084, -0.799956, -0.0487324, 1.89051, -0.872665, -0.0861013, -0.0556453, -0.785294, 0.000522515, 1.87785, -0.872665, 0.0719266, 0.0120855, -0.0164255, 0.630104, 0, 0.223516, -1.13428, 0.272803, -0.797309, 0.0179779, 0.624253, -6.93889e-18, -0.213658, -1.12858, -0.281698, -0.80026, -0.0106817;
    }
    
    std::vector<Eigen::VectorXd> qs = {q0, q1, q2, q3};
    
    q_init = qs.at(INIT_INDEX);
    q_goal = qs.at(GOAL_INDEX);
    
    if(SCENARIO == 1){
        if(INIT_INDEX == 0) activeEEsInit = {L_FOOT, R_FOOT};
        else activeEEsInit = {L_HAND_C, R_HAND_C, L_FOOT, R_FOOT};
        
        activeEEsGoal = {L_HAND_C, R_HAND_C, L_FOOT, R_FOOT};
        allowedEEs = {L_HAND_C, R_HAND_C, L_FOOT, R_FOOT};
    }
    if(SCENARIO == 2){
        if(INIT_INDEX == 0) activeEEsInit = {L_FOOT, R_FOOT};
        else activeEEsInit = {L_HAND_C, R_HAND_C, L_FOOT, R_FOOT};
        
        activeEEsGoal = {L_HAND_C, R_HAND_C, L_FOOT, R_FOOT};
        allowedEEs = {L_HAND_C, R_HAND_C, L_FOOT, R_FOOT};
    }
    if(SCENARIO == 3){
        if(INIT_INDEX == 0) activeEEsInit = {L_FOOT, R_FOOT};
        else activeEEsInit = {L_HAND_D, R_HAND_D, L_FOOT, R_FOOT};
        
        activeEEsGoal = {L_HAND_D, R_HAND_D, L_FOOT, R_FOOT};
        allowedEEs = {L_HAND_D, R_HAND_D, L_FOOT, R_FOOT};
    }
    
    /////////////////////////////////////////////////
    _model->setJointPosition(q_init);
    _model->update();

    _start_model->setJointPosition(q_init);
    _start_model->update();

    _goal_model->setJointPosition(q_goal);
    _goal_model->update();
    //////////////////////////////////////////////////////////////////////////////////////////  

    _pc_manager = std::make_shared<XBot::Planning::PointCloudManager>(_n);
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
        //qmax.head<6>() << 1.0, 1.0, 1.0, 2*M_PI, 2*M_PI, 2*M_PI;
        //qmin.head<6>() << -qmax.head<6>();

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
    _vc_context.planning_scene->acm.setEntry("RBall", "<octomap>", true);
    _vc_context.planning_scene->acm.setEntry("RFoot", "<octomap>", true);
    _vc_context.planning_scene->acm.setEntry("LWrMot2", "<octomap>", true);
    _vc_context.planning_scene->acm.setEntry("LWrMot3", "<octomap>", true);
    _vc_context.planning_scene->acm.setEntry("RWrMot2", "<octomap>", true);
    _vc_context.planning_scene->acm.setEntry("RWrMot3", "<octomap>", true);
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

//    _contact_pub = _nh.advertise<multi_contact_planning::SetContactFrames>("contacts", 10, true);

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

        auto ik_solver_NSPG = std::make_shared<Planning::PositionCartesianSolver>(ci);


        _NSPG = std::make_shared<Planning::NSPG>(ik_solver_NSPG, _vc_context);

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

bool PlannerExecutor::goal_sampler_service(multi_contact_planning::CartesioGoal::Request &req, multi_contact_planning::CartesioGoal::Response &res)
{
    std::vector<std::string> active_tasks;
    std::vector<Eigen::Vector3d> pos_ref;
    
//     Eigen::VectorXd c(n_dof);
//     c << 1.15667, 0.0227583, 1.65702, -0.533903, 0.0705821, 0.286732, 0.264738, -1.28463, -0.558505, 1.2229, -0.151601, -0.261799, 0.187707, -0.85183, -0.350576, 0.782062, -0.088001, 0.0891286, 0.418645, 0.158891, 1.37562, 3.09792, 2.24523, -0.0254683, -1.59168, -0.0676732, 2.50583, -0.19434, -0.925812, -1.31944, -1.61386, -0.61326, 0.878791, 0.192605;
//     _goal_model->setJointPosition(c);
//     _goal_model->update();
//     _model->setJointPosition(c);
//     _model->update();
//      
//        
//     Eigen::Affine3d T;
//     _NSPG->getIKSolver()->getCI()->getCurrentPose("r_sole", T);
//     std::cout << "RF = " << T.translation() << std::endl;
//     _NSPG->getIKSolver()->getCI()->getCurrentPose("l_sole", T);
//     std::cout << "LF = " << T.translation() << std::endl;
//     _NSPG->getIKSolver()->getCI()->getCurrentPose("r_ball_tip_d", T);
//     std::cout << "RH = " << T.translation() << std::endl;
//     _NSPG->getIKSolver()->getCI()->getCurrentPose("l_ball_tip_d", T);
//     std::cout << "LH = " << T.translation() << std::endl;
//  
    active_tasks.clear();
    active_tasks.push_back("l_ball_tip_d");
    active_tasks.push_back("r_ball_tip_d");
    active_tasks.push_back("l_sole");
    active_tasks.push_back("r_sole");
     
    //LH
    pos_ref.push_back(Eigen::Vector3d(1.9, 0.3, 2.0));
    //RH
    pos_ref.push_back(Eigen::Vector3d(1.9, -0.3, 2.0));
    //LF
    pos_ref.push_back(Eigen::Vector3d(1.3, 0.15, 0.8));
    //RF
    pos_ref.push_back(Eigen::Vector3d(1.3, -0.15, 0.8));
    
    // create stance
    Stance sigma;
    for(int i = 0; i < active_tasks.size(); i++){
        Eigen::Vector3d n_i = getNormalAtPoint(pos_ref[i]);
        Eigen::Affine3d T_i;
        T_i.translation() = pos_ref[i];
        T_i.linear() = generateRotationAroundAxis(getTaskEndEffectorName(active_tasks[i]), n_i);
        Eigen::Vector3d F_i(0.0, 0.0, 0.0);
        std::shared_ptr<Contact> c = std::make_shared<Contact>(getTaskEndEffectorName(active_tasks[i]), T_i, F_i, n_i);
        sigma.addContact(c);
    }
    
    Eigen::VectorXd c(n_dof);
    _model->getRobotState("home", c);
    
    Eigen::VectorXd qRand;
    Eigen::VectorXd qMin, qMax;
    _model->getJointLimits(qMin, qMax);
    qRand.setRandom(n_dof); // uniform in -1 < x < 1
    qRand = (qRand.array() + 1)/2.0; // uniform in 0 < x < 1
    qRand = qMin + qRand.cwiseProduct(qMax - qMin); // uniform in qmin < x < qmax
    qRand.head<6>().setRandom(); // we keep virtual joints between -1 and 1 (todo: improve)
    qRand.head<6>().tail<3>() *= M_PI;
    
    //c = qRand;
    
    Configuration qCurr; 
    Configuration qNew;
    qCurr.setFBPosition(c.segment(0,3));
    qCurr.setFBOrientation(c.segment(3,3));
    qCurr.setJointValues(c.tail(n_dof-6));
    
    bool resIKCS = computeIKandCS(sigma, sigma, qCurr, qNew);
    if(resIKCS){
        std::cout << "--------------- GS SUCCESS ---------------" << std::endl;
        
        _NSPG->getIKSolver()->getModel()->getJointPosition(c);
        _NSPG->_rspub->publishTransforms(ros::Time::now(), "/planner");
        _goal_model->setJointPosition(c);
        _goal_model->update();

        for(int z = 0; z < c.rows(); z++) std::cout << c(z) << ", ";
        std::cout << " " << std::endl;
        
        return true;
        
    }
       
    std::cout << "--------------- GS FAIL ---------------" << std::endl;
    return false;   
}

/*
bool PlannerExecutor::goal_sampler_service(multi_contact_planning::CartesioGoal::Request &req, multi_contact_planning::CartesioGoal::Response &res)
{
    std::vector<std::string> active_tasks;
    std::vector<Eigen::Affine3d> ref_tasks;
    
    Eigen::Affine3d T_ref;
    Eigen::Matrix3d rot_ref = Eigen::Matrix3d::Identity(3,3);
    Eigen::Vector3d pos_ref;
    
    active_tasks.clear();
    active_tasks.push_back("l_ball_tip_d");
    active_tasks.push_back("r_ball_tip_d");
    active_tasks.push_back("l_sole");
    active_tasks.push_back("r_sole");
 
    //LH
    //T_ref.translation() << 1.1, 0.3, 0.8;
    T_ref.translation() << 1.25, 0.3, 1.0;
    T_ref.linear() << -1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, -1.0;
    ref_tasks.push_back(T_ref);
    //RH
    //T_ref.translation() << 1.1, -0.3, 0.8;
    T_ref.translation() << 1.25, -0.3, 1.0;
    T_ref.linear() <<  -1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, -1.0;
    ref_tasks.push_back(T_ref); 
    //LF
    //T_ref.translation() << 0.8, 0.2, 0.0;
    T_ref.translation() << 1.0, 0.2, 0.2;
    T_ref.linear() <<  1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0;
    ref_tasks.push_back(T_ref);
    //RF
    //T_ref.translation() << 0.8, -0.2, 0.0;
    T_ref.translation() << 1.0, -0.2, 0.2;
    T_ref.linear() <<  1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0;
    ref_tasks.push_back(T_ref);
    
    // create stance
    Stance sigma;
    for(int i = 0; i < active_tasks.size(); i++){
        //Eigen::Vector3d n_i = getNormalAtPoint(pos_ref[i]);
        Eigen::Vector3d n_i(0.0, 0.0, 0.0);
        Eigen::Affine3d T_i;
        //T_i.translation() = pos_ref[i];
        T_i.linear() = generateRotationAroundAxis(getTaskEndEffectorName(active_tasks[i]), n_i);
        Eigen::Vector3d F_i(0.0, 0.0, 0.0);
        std::shared_ptr<Contact> c = std::make_shared<Contact>(getTaskEndEffectorName(active_tasks[i]), T_i, F_i, n_i);
        sigma.addContact(c);
    }
    

    
    std::vector<std::string> all_tasks;
    all_tasks.push_back("com");
    all_tasks.push_back("l_ball_tip_d");
    all_tasks.push_back("r_ball_tip_d");
    all_tasks.push_back("TCP_L");
    all_tasks.push_back("TCP_R");
    all_tasks.push_back("l_sole");
    all_tasks.push_back("r_sole");

    for(int i = 0; i < all_tasks.size(); i++){
        std::vector<std::string>::iterator it = std::find(active_tasks.begin(), active_tasks.end(), all_tasks[i]);               
        if(it == active_tasks.end()){
            _NSPG->getIKSolver()->getCI()->setActivationState(all_tasks[i], XBot::Cartesian::ActivationState::Disabled);
        }
        else{
            _NSPG->getIKSolver()->getCI()->setActivationState(all_tasks[i], XBot::Cartesian::ActivationState::Enabled);
            int index = it - active_tasks.begin();
            _NSPG->getIKSolver()->getCI()->setPoseReference(all_tasks.at(i), ref_tasks[index]);
        }
    }

    Eigen::VectorXd qHome; 
    _model->getRobotState("home", qHome);
    
    Eigen::VectorXd qRand;
    Eigen::VectorXd qMin, qMax;
    _model->getJointLimits(qMin, qMax);
    qRand.setRandom(n_dof); // uniform in -1 < x < 1
    qRand = (qRand.array() + 1)/2.0; // uniform in 0 < x < 1
    qRand = qMin + qRand.cwiseProduct(qMax - qMin); // uniform in qmin < x < qmax
    qRand.head<6>().setRandom(); // we keep virtual joints between -1 and 1 (todo: improve)
    qRand.head<6>().tail<3>() *= M_PI;
    
    _NSPG->getIKSolver()->getModel()->setJointPosition(qRand);
    XBot::JointNameMap jmap;
    _NSPG->getIKSolver()->getModel()->eigenToMap(qHome, jmap);
    _NSPG->getIKSolver()->getCI()->setReferencePosture(jmap);
    _NSPG->getIKSolver()->getModel()->update();
    
//      _goal_model->setJointPosition(qHome);
//      return false;
    
    
    Eigen::VectorXd c(n_dof);

    if(_NSPG->getIKSolver()->solve()){
        _NSPG->getIKSolver()->getModel()->getJointPosition(c);
        _NSPG->_rspub->publishTransforms(ros::Time::now(), "/planner");
        _goal_model->setJointPosition(c);
        _goal_model->update();

        for(int z = 0; z < c.rows(); z++) std::cout << c(z) << ", ";
        std::cout << " " << std::endl;

        return true;
    }    

    return false;    

}
*/


/*
bool PlannerExecutor::goal_sampler_service(multi_contact_planning::CartesioGoal::Request &req, multi_contact_planning::CartesioGoal::Response &res)
{
    std::vector<std::string> active_tasks;
    std::vector<Eigen::Affine3d> ref_tasks;
    
    Eigen::Affine3d T_ref;
    Eigen::Matrix3d rot_ref = Eigen::Matrix3d::Identity(3,3);
    Eigen::Vector3d pos_ref;
    
    active_tasks.clear();
      active_tasks.push_back("TCP_L");
      active_tasks.push_back("TCP_R");
    active_tasks.push_back("l_sole");
    active_tasks.push_back("r_sole");
 
    //LH
    T_ref.translation() << 0.5, 0.8, 1.5;
    T_ref.linear() << 0.0, -1.0, 0.0,
                        0.0, 0.0, 1.0,
                        -1.0, 0.0, 0.0;
    ref_tasks.push_back(T_ref);
    //RH
    T_ref.translation() << 0.5, -0.8, 1.5;
    T_ref.linear() <<  0.0, 1.0, 0.0,
                        0.0, 0.0, -1.0,
                        -1.0, 0.0, 0.0;
    ref_tasks.push_back(T_ref); 
    //LF
    //T_ref.translation() << 0.4, 0.15, 0.0;
    T_ref.translation() << 0.4, 0.8, 0.4;
    T_ref.linear() <<  0.0, -1.0, 0.0,
                        0.0, 0.0, -1.0,
                        1.0, 0.0, 0.0;
    ref_tasks.push_back(T_ref);
    //RF
    //T_ref.translation() << 0.4, -0.15, 0.0;
    T_ref.translation() << 0.4, -0.8, 0.4;
    T_ref.linear() <<  0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0,
                        1.0, 0.0, 0.0;
    ref_tasks.push_back(T_ref);

    
    std::vector<std::string> all_tasks;
    all_tasks.push_back("com");
    all_tasks.push_back("TCP_L");
    all_tasks.push_back("TCP_R");
    all_tasks.push_back("l_sole");
    all_tasks.push_back("r_sole");

    for(int i = 0; i < all_tasks.size(); i++){
        std::vector<std::string>::iterator it = std::find(active_tasks.begin(), active_tasks.end(), all_tasks[i]);               
        if(it == active_tasks.end()){
            _NSPG->getIKSolver()->getCI()->setActivationState(all_tasks[i], XBot::Cartesian::ActivationState::Disabled);
        }
        else{
            _NSPG->getIKSolver()->getCI()->setActivationState(all_tasks[i], XBot::Cartesian::ActivationState::Enabled);
            int index = it - active_tasks.begin();
            _NSPG->getIKSolver()->getCI()->setPoseReference(all_tasks.at(i), ref_tasks[index]);
        }
    }

    Eigen::VectorXd qHome; 
    _model->getRobotState("home", qHome);
    
    Eigen::VectorXd qRand;
    Eigen::VectorXd qMin, qMax;
    _model->getJointLimits(qMin, qMax);
    qRand.setRandom(n_dof); // uniform in -1 < x < 1
    qRand = (qRand.array() + 1)/2.0; // uniform in 0 < x < 1
    qRand = qMin + qRand.cwiseProduct(qMax - qMin); // uniform in qmin < x < qmax
    qRand.head<6>().setRandom(); // we keep virtual joints between -1 and 1 (todo: improve)
    qRand.head<6>().tail<3>() *= M_PI;
    
    _NSPG->getIKSolver()->getModel()->setJointPosition(qRand);
    XBot::JointNameMap jmap;
    _NSPG->getIKSolver()->getModel()->eigenToMap(qHome, jmap);
    _NSPG->getIKSolver()->getCI()->setReferencePosture(jmap);
    _NSPG->getIKSolver()->getModel()->update();
    
//      _goal_model->setJointPosition(qHome);
//      return false;
    
    
    Eigen::VectorXd c(n_dof);

    if(_NSPG->getIKSolver()->solve()){
        _NSPG->getIKSolver()->getModel()->getJointPosition(c);
        _NSPG->_rspub->publishTransforms(ros::Time::now(), "/planner");
        _goal_model->setJointPosition(c);
        _goal_model->update();

        for(int z = 0; z < c.rows(); z++) std::cout << c(z) << ", ";
        std::cout << " " << std::endl;

        return true;
    }    

    return false;    

}
*/
 
 
/*
bool PlannerExecutor::goal_sampler_service(multi_contact_planning::CartesioGoal::Request &req,
                                           multi_contact_planning::CartesioGoal::Response &res)
{

    std::vector<std::string> active_tasks;
    std::vector<Eigen::Affine3d> ref_tasks;
    
    active_tasks.push_back("TCP_L");
    active_tasks.push_back("TCP_R");
    active_tasks.push_back("l_sole");
    active_tasks.push_back("r_sole");

    Eigen::Affine3d T_ref;
    Eigen::Matrix3d rot_ref = Eigen::Matrix3d::Identity(3,3);
    Eigen::Vector3d pos_ref;
    
    int qIndex = 3;
   
    if(qIndex == 0){
        ///// generation of q0 //////////////////////////////////////////////
        active_tasks.clear();
        active_tasks.push_back("l_sole");
        active_tasks.push_back("r_sole");
        _NSPG->getIKSolver()->getCI()->setActivationState("TCP_L", XBot::Cartesian::ActivationState::Disabled);
        _NSPG->getIKSolver()->getCI()->setActivationState("TCP_R", XBot::Cartesian::ActivationState::Disabled);
        //LF
        T_ref.translation() << 0.0, 0.15, 0.0;
        T_ref.linear() = rot_ref;
        ref_tasks.push_back(T_ref);
        //RF
        T_ref.translation() << 0.0, -0.15, 0.0;
        T_ref.linear() = rot_ref;
        ref_tasks.push_back(T_ref);
    }
    else if(qIndex == 1){
        ///// generation of q1 //////////////////////////////////////////////
        //LH
        T_ref.translation() << 1.1, 0.1, 0.0;
        T_ref.linear() << -1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, -1.0;
        ref_tasks.push_back(T_ref);
        //RH
        T_ref.translation() << 1.1, -0.1, 0.0;
        T_ref.linear() << -1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, -1.0;
        ref_tasks.push_back(T_ref);
        //LF
        T_ref.translation() << 0.4, 0.15, 0.0;
        T_ref.linear() = rot_ref;
        ref_tasks.push_back(T_ref);
        //RF
        T_ref.translation() << 0.4, -0.15, 0.0;
        T_ref.linear() = rot_ref;
        ref_tasks.push_back(T_ref);
    }
    else if(qIndex == 2){
        ///// generation of q2 //////////////////////////////////////////////
        //LH
        T_ref.translation() << 2.1, 0.4, 0.0;
        T_ref.linear() << -1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, -1.0;
        ref_tasks.push_back(T_ref);
        //RH
        T_ref.translation() << 2.1, -0.4, 0.0;
        T_ref.linear() << -1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, -1.0;
        ref_tasks.push_back(T_ref);
        //LF
        T_ref.translation() << 1.0, 0.3, 0.0;
        T_ref.linear() = rot_ref;
        ref_tasks.push_back(T_ref);
        //RF
        T_ref.translation() << 1.0, -0.3, 0.0;
        T_ref.linear() = rot_ref;
        ref_tasks.push_back(T_ref);
    }
    else{
        ///// generation of q3 //////////////////////////////////////////////
        //LH
        T_ref.translation() << 2.5, 0.3, 1.4;
        T_ref.linear() << 0.0, 0.0, 1.0,
                            0.0, 1.0, 0.0,
                            -1.0, 0.0, 0.0;
        ref_tasks.push_back(T_ref);
        //RH
        T_ref.translation() << 2.5, -0.3, 1.4;
        T_ref.linear() << 0.0, 0.0, 1.0,
                            0.0, 1.0, 0.0,
                            -1.0, 0.0, 0.0;
        ref_tasks.push_back(T_ref);
        //LF
        T_ref.translation() << 1.5, 0.2, 0.0;
        T_ref.linear() = rot_ref;
        ref_tasks.push_back(T_ref);
        //RF
        T_ref.translation() << 1.5, -0.2, 0.0;
        T_ref.linear() = rot_ref;
        ref_tasks.push_back(T_ref);
    }
   
    
    // set references
    std::vector<std::string> all_tasks;
    all_tasks.push_back("com");
    all_tasks.push_back("TCP_L");
    all_tasks.push_back("TCP_R");
    all_tasks.push_back("l_sole");
    all_tasks.push_back("r_sole");

    _NSPG->getIKSolver()->getCI()->setActivationState(all_tasks[0], XBot::Cartesian::ActivationState::Disabled);
    for(int i = 1; i < all_tasks.size(); i++){
        std::vector<std::string>::iterator it = std::find(active_tasks.begin(), active_tasks.end(), all_tasks[i]);               
        int index = it - active_tasks.begin();
        _NSPG->getIKSolver()->getCI()->setPoseReference(all_tasks.at(i), ref_tasks[index]);
    }

    Eigen::VectorXd qHome; 
    _model->getRobotState("home", qHome);
    
    Eigen::VectorXd qQuad(n_dof);
    qQuad << 0.34515884431887384, -0.06591591904073339, 0.7271543349204505, 2.772752261057329, 1.694067260637883, -2.8326452668824484, 0.02082929860422072, -1.7787860844940504, -0.036421660785962574, 0.9996204693896318, -0.6689316045377748, 0.04679752671173139, -0.0047857492997280225, -1.7034599738559666, -0.06227672563131584, 0.890601586605412, -0.6349870611535411, 0.04271151312504321, -0.02360545515374067, 0.032760740733259075, -0.707631719076811, 0.659625032411939, 0.04654837196558426, -0.9962331912723077, 0.6763772547285989, 0.44353465292278027, -0.2790720832627141, -0.6992796605078045, -0.6140390267081726, -0.10013692237630738, -0.9404489978405196, -0.6420967750257626, 0.3194200132256253, 0.17978778269015258;
    
    Eigen::VectorXd qRand;
    Eigen::VectorXd qMin, qMax;
    _model->getJointLimits(qMin, qMax);
    qRand.setRandom(n_dof); // uniform in -1 < x < 1
    qRand = (qRand.array() + 1)/2.0; // uniform in 0 < x < 1
    qRand = qMin + qRand.cwiseProduct(qMax - qMin); // uniform in qmin < x < qmax
    qRand.head<6>().setRandom(); // we keep virtual joints between -1 and 1 (todo: improve)
    qRand.head<6>().tail<3>() *= M_PI;
    
//     if(qIndex == 1 || qIndex == 2) _NSPG->getIKSolver()->getModel()->setJointPosition(qQuad);
//     else _NSPG->getIKSolver()->getModel()->setJointPosition(qHome); 
    _NSPG->getIKSolver()->getModel()->setJointPosition(qRand);
    _NSPG->getIKSolver()->getModel()->update();

    double time_budget = GOAL_SAMPLER_TIME_BUDGET;
    Eigen::VectorXd c(n_dof);

    _NSPG->getIKSolver()->solve();
    _NSPG->_rspub->publishTransforms(ros::Time::now(), "/planner");
    
    
    if(_NSPG->sample(time_budget)){
        _NSPG->getIKSolver()->getModel()->getJointPosition(c);
        _goal_model->setJointPosition(c);
        _goal_model->update();
    
        for(int z = 0; z < c.rows(); z++) std::cout << c(z) << ", ";
        std::cout << " " << std::endl;
    
        return true;
    }    
    
    return false;

}
*/

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
    qInit.setFBPosition(Eigen::Vector3d(q_init(0), q_init(1), q_init(2)));
    qInit.setFBOrientation(Eigen::Vector3d(q_init(3), q_init(4), q_init(5)));
    qInit.setJointValues(q_init.tail(n_dof-6));
    
    // retrieve goal
    Configuration qGoal;
    qGoal.setFBPosition(Eigen::Vector3d(q_goal(0), q_goal(1), q_goal(2)));
    qGoal.setFBOrientation(Eigen::Vector3d(q_goal(3), q_goal(4), q_goal(5)));
    qGoal.setJointValues(q_goal.tail(n_dof-6));
    
    // construct the environment description
    Eigen::MatrixXd pointCloud = _pc_manager->getPointCloud();
    Eigen::MatrixXd pointNormals = _pc_manager->getNormals();

    
    // plan a solution
    std::vector<Stance> sigmaList;
    std::vector<Configuration> qList;
    bool sol_found;

    bool runPlanner = true; 

    if(index_config == -1){

        if(runPlanner){
            // create/initialize the planner
            Planner* planner = new Planner(qInit, activeEEsInit, qGoal, activeEEsGoal, pointCloud, pointNormals, allowedEEs, _model, _NSPG, _vc_context, _nh);
            std::cout << "planner created!" << std::endl;

            // run planner
            float t_elapsed = 0.0;
            auto t_start_chrono = std::chrono::steady_clock::now();
            planner->run();
            std::cout << "Planning completed!" << std::endl;
            auto t_curr_chrono = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t_start_chrono).count();
            t_elapsed = (float)t_curr_chrono / 1000.0;
            std::cout << "Planning Time:: " << t_elapsed << std::endl;

            // retrieve solution
            sigmaList.clear();
            qList.clear();
            sol_found = planner->retrieveSolution(sigmaList, qList);
            if(sol_found) std::cout << "Solution FOUND!" << std::endl;
            else std::cout << "Solution NOT FOUND!" << std::endl;
            std::cout << "sigmaList.size() = " << sigmaList.size() << std::endl;
            std::cout << "qList.size() = " << qList.size() << std::endl;
            writeOnFileConfigs(qList, "qList");
            writeOnFileStances(sigmaList, "sigmaList");
        }
        else{
            sigmaList.clear();
            qList.clear();
            readFromFileConfigs(qList, "qList");
            readFromFileStances(sigmaList, "sigmaList");
            sol_found = true;
            
            
            /////////////////////////////////////
            // create/initialize the planner  
            Planner* planner = new Planner(qInit, activeEEsInit, qGoal, activeEEsGoal, pointCloud, pointNormals, allowedEEs, _model, _NSPG, _vc_context, _nh);
            std::cout << "planner created!" << std::endl;
            planner->checkSolutionCS(sigmaList, qList);
           
        }

/*        _vc_context.planning_scene->acm.setEntry("RBall", "<octomap>", true);
        _vc_context.planning_scene->acm.setEntry("LBall", "<octomap>", true);
        _vc_context.planning_scene->acm.setEntry("LFoot", "<octomap>", true);
        _vc_context.planning_scene->acm.setEntry("RFoot", "<octomap>", true); */

    }

    // this is for DEBUGGING
    if(index_config == -1 && sol_found){
        for(int i = 0; i < qList.size(); i++){
            Configuration q = qList.at(i);
            Eigen::VectorXd c(n_dof);
            c.segment(0,3) = q.getFBPosition();
            c.segment(3,3) = q.getFBOrientation();
            c.tail(n_dof-6) = q.getJointValues();
            plan.push_back(c);
        }
        index_config++;
    }

//    multi_contact_planning::SetContactFrames contact;
    if(plan.size() > 0){
        _goal_model->setJointPosition(plan[index_config]);
        _goal_model->update();

//        contact.action = multi_contact_planning::SetContactFrames::SET;
//        contact.frames_in_contact = sigmaList[index_config].getContacts();

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

Eigen::Vector3d PlannerExecutor::getNormalAtPoint(Eigen::Vector3d p){
    Eigen::MatrixXd pointCloud = _pc_manager->getPointCloud();
    Eigen::MatrixXd pointNormals = _pc_manager->getNormals();
    
    double dMin = std::numeric_limits<double>::max();
    double iMin = -1;
    for(int i = 0; i < pointCloud.rows(); i++){
        Eigen::Vector3d p_i = pointCloud.row(i).transpose();
        double d = euclideanDistance(p_i, p);
        if(d < dMin){
            dMin = d;
            iMin = i;
        }
    }

    Eigen::Vector3d nC = pointNormals.row(iMin).transpose();

    return nC;
}

bool PlannerExecutor::computeIKandCS(Stance sigmaSmall, Stance sigmaLarge, Configuration qNear, Configuration &qNew){
    
    // build references
    std::vector<std::string> active_tasks;
    std::vector<Eigen::Affine3d> ref_tasks;
    for(int i = 0; i < sigmaLarge.getSize(); i++){
        EndEffector ee = sigmaLarge.getContact(i)->getEndEffectorName();
        active_tasks.push_back(getTaskStringName(ee));
        ref_tasks.push_back(sigmaLarge.retrieveContactPose(ee));
    }
    
    // set references
    std::vector<std::string> all_tasks = {"r_sole", "l_sole", "TCP_R", "TCP_L", "l_ball_tip_d", "r_ball_tip_d"};
    _NSPG->getIKSolver()->getCI()->setActivationState("com", XBot::Cartesian::ActivationState::Disabled); //FIXME if it is not in the stack this is not needed  
    for(int i = 0; i < all_tasks.size(); i++){
        std::vector<std::string>::iterator it = std::find(active_tasks.begin(), active_tasks.end(), all_tasks[i]);
        if(it == active_tasks.end()){
            _NSPG->getIKSolver()->getCI()->setActivationState(all_tasks[i], XBot::Cartesian::ActivationState::Disabled); 
        }
        else{ 
            _NSPG->getIKSolver()->getCI()->setActivationState(all_tasks[i], XBot::Cartesian::ActivationState::Enabled); 
            int index = it - active_tasks.begin();
            _NSPG->getIKSolver()->getCI()->setPoseReference(all_tasks.at(i), ref_tasks[index]);
        }
    }

    // set postural
    Eigen::VectorXd cPrev(n_dof);
    Eigen::Vector3d posFB = qNear.getFBPosition();
    Eigen::Vector3d rotFB = qNear.getFBOrientation();
    cPrev.segment(0,3) = posFB;
    cPrev.segment(3,3) = rotFB;
    cPrev.tail(n_dof-6) = qNear.getJointValues();
    _NSPG->getIKSolver()->getModel()->setJointPosition(cPrev);
    _NSPG->getIKSolver()->getModel()->update();

    // search IK solution (joint limits)
    double time_budget = GOAL_SAMPLER_TIME_BUDGET;
    Eigen::VectorXd c(n_dof);
    if (!_NSPG->getIKSolver()->solve()){
        //foutLogMCP << "STOP BEFORE NSPG" << std::endl;
        return false;
    }
    
    // refine IK solution (balance and self-collisions)
    _NSPG->_rspub->publishTransforms(ros::Time::now(), "/planner");
   
    if(!_NSPG->sample(time_budget, sigmaSmall, sigmaLarge))
    {
        _NSPG->getIKSolver()->getModel()->getJointPosition(c);
        qNew.setFBPosition(c.segment(0,3));
        qNew.setFBOrientation(c.segment(3,3));
        qNew.setJointValues(c.tail(n_dof-6));
        
        return false;
    }
    else
    {
        _NSPG->getIKSolver()->getModel()->getJointPosition(c);
        qNew.setFBPosition(c.segment(0,3));
        qNew.setFBOrientation(c.segment(3,3));
        qNew.setJointValues(c.tail(n_dof-6));
        
        return true;
    }
}


