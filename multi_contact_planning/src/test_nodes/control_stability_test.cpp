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

XBot::Cartesian::CartesianInterfaceImpl::Ptr ci;

int ind = 0;
std::vector<Configuration> qList;
Eigen::VectorXd q;

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
    auto task = ci->getTask(name + "_wrench");
    auto task_force = std::dynamic_pointer_cast<XBot::Cartesian::acceleration::ForceTask>(task);
    if (task_force == nullptr)
        ROS_ERROR("Something went wrong while casting to 'ForceTask'");
    task_force->setForceReference(force);

    task = ci->getTask(name + "_fc");
    auto task_fc = std::dynamic_pointer_cast<XBot::Cartesian::acceleration::FrictionCone>(task);
    if (task_fc == nullptr)
        ROS_ERROR("Something went wrong while casting to 'FrictionCone'");
    task_fc->setContactRotationMatrix(pose.linear());

    Eigen::Vector6d fmin_feet, fmax_feet, fmin_hands, fmax_hands;
    fmin_feet << -1000., -1000., -1000., -1000., -1000., -1000.;
    fmax_feet = -fmin_feet;
    fmin_hands << -1000., -1000., -1000., 0., 0., 0.;
    fmax_hands = -fmin_hands;

    task = ci->getTask(name + "_F_lims");
    auto task_flims = std::dynamic_pointer_cast<XBot::Cartesian::acceleration::ForceLimits>(task);
    if (task_flims == nullptr)
        ROS_ERROR("Something went wrong while casting to 'ForceLimits'");
    if (name == "l_sole" || name == "r_sole")
        task_flims->setLimits(fmin_feet, fmax_feet);
    else
        task_flims->setLimits(fmin_hands, fmax_hands);
}


bool change_service(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
    ind++;
    if (ind == qList.size())
        ind = 0;

    std::cout << "Picking configuration # " << ind << std::endl;

    // Update configuration and set it to the ModelInterface
    std::cout << "Updating configuration..." << std::endl;
    Configuration c = qList[ind];
    q.segment(0,3) = c.getFBPosition();
    q.segment(3,3) = c.getFBOrientation();
    q.tail(n_dof-6) = c.getJointValues();

    model->setJointPosition(q);
    model->update();

    std::cout << "Actual configuration is: " << q.transpose() << std::endl;

    // Update stance and ci tasks
    std::cout << "Updating stance..." << std::endl;
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

        auto task = ci->getTask(name + "_F_lims");
        auto task_flims = std::dynamic_pointer_cast<acceleration::ForceLimits>(task);
        if (task_flims == nullptr)
            ROS_ERROR("Something went wrong while casting to 'ForceLimits'");
        task_flims->setLimits(Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero());
    }
    std::cout << "Actual stance is: " << std::endl;

    return true;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_stability_test_node");
    ros::NodeHandle nh("");
    ros::ServiceServer srv = nh.advertiseService("change_configuration", &change_service);
    ros::ServiceClient cli = nh.serviceClient<std_srvs::Empty>("change_configuration");

    // Load ModelInterface from param server
    auto cfg = XBot::ConfigOptionsFromParamServer();
    model = XBot::ModelInterface::getModel(cfg);

    XBot::Cartesian::Utils::RobotStatePublisher rs_pub(model);

    n_dof = model->getJointNum();

    // Read qList.txt and sigmaList.txt
    readFromFileConfigs(qList, "qList");
    readFromFileStances(stanceList, "stanceList");

    // Load problem
    XBot::Cartesian::RosServerClass::Ptr rsc;
    std::string problem_description_string;
    if(!nh.getParam("problem_description", problem_description_string))
    {
        ROS_ERROR("problem_description not provided!");
        throw std::runtime_error("problem_description not provided!");
    }

    auto ik_yaml_goal = YAML::Load(problem_description_string);

    double ci_period = 1.0;
    auto ci_ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(ci_period),
                model);

    auto ik_prob = ProblemDescription(ik_yaml_goal, ci_ctx);

    ci = CartesianInterfaceImpl::MakeInstance("OpenSot", ik_prob, ci_ctx);

    rsc = std::make_shared<RosServerClass>(ci);

    // Take the first configuration from the qList
//    cli.waitForExistence();
//    std_srvs::Empty empty;
//    cli.call(empty);

    double time = 0;
    int r = 30;
    ros::Rate rate(r);

    while (ros::ok())
    {
        if (!ci->update(time, 1./r))
            ROS_WARN("unable to solve!");
        time += 1./r;

        rsc->run();
        rs_pub.publishTransforms(ros::Time::now(), "ci");

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
