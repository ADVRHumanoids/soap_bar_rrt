#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/service.h>
#include <fstream>

#include <std_srvs/Empty.h>

#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/ConfigOptions.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include <cartesio_acceleration_support/Force.h>
#include <cartesio_acceleration_support/ForceLimits.h>
#include <cartesio_acceleration_support/FrictionCone.h>
#include <cartesio_acceleration_support/CoP.h>

#include "planner/multi_contact/Configuration.hpp"
#include "planner/multi_contact/Stance.hpp"
#include "planner/multi_contact/Contact.hpp"
#include "planner/multi_contact/enum.h"

using namespace XBot::Cartesian;

std::string env(std::getenv("ROBOTOLOGY_ROOT"));

XBot::ModelInterface::Ptr model;
int n_dof;

XBot::Cartesian::CartesianInterfaceImpl::Ptr ci, ci_com;

int ind = 0;
std::vector<Configuration> qList;
Eigen::VectorXd q, q_jnt;

double time_;

std::vector<Stance>  stanceList;

void readFromFileConfigs(std::vector<Configuration> &qList, std::string fileName){
    std::string filePrefix = env + "/external/soap_bar_rrt/multi_contact_planning/PlanningData/";
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
    std::cout << "Loaded " << qList.size() << " configurations" << std::endl;
}

void readFromFileStances(std::vector<Stance> &sigmaList, std::string fileName){
    std::string filePrefix = env + "/external/soap_bar_rrt/multi_contact_planning/PlanningData/";
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

            std::shared_ptr<Contact> c = std::make_shared<Contact>(ee, T, F, n);
            sigma.addContact(c);
        }

        sigmaList.push_back(sigma);
    }
    std::cout << "Loaded " << sigmaList.size() << " stances" << std::endl;
}

void assign_stance(Contact::Ptr contact)
{
    EndEffector ee = contact->getEndEffectorName();
    Eigen::Affine3d pose = contact->getPose();
    Eigen::VectorXd force = contact->getForce();

    std::string name;
    switch (ee){
        case (0):
            name = "l_ball_tip";
            break;
        case(1):
            name = "r_ball_tip";
            break;
        case(4):
            name = "l_sole";
            break;
        case(5):
            name = "r_sole";
            break;
    }

    // force reference
    auto task = ci->getTask("force_" + name);
    auto task_force = std::dynamic_pointer_cast<XBot::Cartesian::acceleration::ForceTask>(task);
    task_force->setForceReference(force);
    std::cout << "Force reference " << name << ": " << task_force->getForceReference().transpose() << std::endl;

    // friction cones
    task = ci->getTask("friction_cone_" + name);
    auto task_fc = std::dynamic_pointer_cast<XBot::Cartesian::acceleration::FrictionCone>(task);
    Eigen::Affine3d T;
    model->getPose(name, T);
    task_fc->setContactRotationMatrix(T.linear());

    // force limits
    Eigen::Vector6d fmin_feet, fmax_feet, fmin_hands, fmax_hands;
    fmin_feet << -1000., -1000., -1000., -1000., -1000., -1000.;
    fmax_feet = -fmin_feet;
    fmin_hands << -1000., -1000., -1000., 0., 0., 0.;
    fmax_hands = -fmin_hands;

    task = ci->getTask("force_lims_" + name);
    auto task_flims = std::dynamic_pointer_cast<XBot::Cartesian::acceleration::ForceLimits>(task);
    if (task_flims == nullptr)
        task_flims->setLimits(fmin_feet, fmax_feet);
    else
        task_flims->setLimits(fmin_hands, fmax_hands);
}


bool change_service(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
    if (ind == qList.size())
        ind = 0;

//    std::cout << "Picking configuration # " << ind << std::endl;

    // Update configuration and set it to the ModelInterface
//    std::cout << "Updating configuration..." << std::endl;
    q.segment(0,3) = qList[ind].getFBPosition();
    q.segment(3,3) = qList[ind].getFBOrientation();
    q.tail(n_dof-6) = qList[ind].getJointValues();

    model->setJointPosition(q);
    model->update();
    ci_com->reset(time_);


//    std::cout << "Actual configuration is: " << q.transpose() << std::endl;

    // Update stance and ci tasks
//    std::cout << "Updating stance..." << std::endl;
    Stance stance = stanceList[ind];
    auto contacts = stance.getContacts();
    for (auto contact : contacts)
    {
        assign_stance(contact);
    }

    // set ForceLimits to zero for non-active contacts
    std::vector<EndEffector> all_ee {EndEffector::L_HAND_C, EndEffector::R_HAND_C, EndEffector::L_FOOT, EndEffector::R_FOOT};
    for (auto contact : contacts)
    {
        std::vector<EndEffector>::iterator it;
        it = std::find(all_ee.begin(), all_ee.end(), contact->getEndEffectorName());
        if (it != all_ee.end())
            all_ee.erase(it);
    }

    for (auto ee : all_ee)
    {
        std::cout << "non_active: " << ee << std::endl;
        std::string name;
        switch (ee){
            case (0):
                name = "l_ball_tip";
                break;
            case(1):
                name = "r_ball_tip";
                break;
            case(4):
                name = "l_sole";
                break;
            case(5):
                name = "r_sole";
                break;
        }

        auto task = ci->getTask("force_lims_" + name);
        auto task_flims = std::dynamic_pointer_cast<acceleration::ForceLimits>(task);
        task_flims->setLimits(Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero());
    }
//    std::cout << "Actual stance is: " << std::endl;
//    stance.print();
    ind++;

    return true;
}

void callback(const sensor_msgs::JointStatePtr& msg)
{
    q_jnt = Eigen::VectorXd::Map(msg->position.data(), msg->position.size());

    model->setJointPosition(q_jnt);
    model->update();
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_stability_test_node");
    ros::NodeHandle nh("");
    ros::ServiceServer srv = nh.advertiseService("change_configuration", &change_service);

    ros::Subscriber q_sub;
    q_sub = nh.subscribe("cartesian_markers/com_markers_solution", 10, callback);

    // Load ModelInterface from param server
    auto cfg = XBot::ConfigOptionsFromParamServer();
    model = XBot::ModelInterface::getModel(cfg);

    XBot::Cartesian::Utils::RobotStatePublisher rs_pub(model);

    n_dof = model->getJointNum();

    // Read qList.txt and sigmaList.txt
    q.resize(n_dof);
    readFromFileConfigs(qList, "qList");
    std::cout << "Configurations loaded successfully!" << std::endl;
    readFromFileStances(stanceList, "sigmaList");
    std::cout << "Stances loaded successfully!" << std::endl;

    // Load problem
    XBot::Cartesian::RosServerClass::Ptr rsc,  rsc_com;
    std::string problem_description_string, problem_description_string_rsc;
    if(!nh.getParam("problem_description", problem_description_string))
    {
        ROS_ERROR("problem_description not provided!");
        throw std::runtime_error("problem_description not provided!");
    }
    if(!nh.getParam("problem_description_rsc", problem_description_string_rsc))
    {
        ROS_ERROR("problem_description_rsc not provided!");
        throw std::runtime_error("problem_description_rsc not provided!");
    }

    auto ik_yaml_goal = YAML::Load(problem_description_string);
    double ci_period = 1.0;
    auto ci_ctx = std::make_shared<Context>(std::make_shared<Parameters>(ci_period), model);
    auto ik_prob = ProblemDescription(ik_yaml_goal, ci_ctx);
    ci = CartesianInterfaceImpl::MakeInstance("OpenSot", ik_prob, ci_ctx);

    auto ik_yaml_goal_com = YAML::Load(problem_description_string_rsc);
    auto ci_ctx_com = std::make_shared<Context>(std::make_shared<Parameters>(ci_period), model);
    auto ik_prob_com = ProblemDescription(ik_yaml_goal_com, ci_ctx_com);
    ci_com = CartesianInterfaceImpl::MakeInstance("OpenSot", ik_prob_com, ci_ctx_com);

    RosServerClass::Options opt;
    opt.ros_namespace = "cartesian_markers";
    opt.tf_prefix = "ci";

    rsc = std::make_shared<RosServerClass>(ci, opt);
    rsc_com = std::make_shared<RosServerClass>(ci_com);

    time_ = 0;
    int r = 30;
    ros::Rate rate(r);

    while (ros::ok())
    {
        if (!ci->update(time_, 1./r))
            ROS_WARN("unable to solve force optimization!");

        ci_com->update(time_, 1./r);

        /* Integrate solution */
        Eigen::VectorXd q_jnt, qdot, qddot;
        model->getJointPosition(q_jnt);
        model->getJointVelocity(qdot);
        model->getJointAcceleration(qddot);

        q_jnt += 1./r * qdot + 0.5 * std::pow(1./r, 2) * qddot;
        qdot += 1./r * qddot;

        model->setJointPosition(q_jnt);
        model->setJointVelocity(qdot);
        model->update();

        time_ += 1./r;

        rsc->run();
        rsc_com->run();

        rs_pub.publishTransforms(ros::Time::now(), "ci");

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
