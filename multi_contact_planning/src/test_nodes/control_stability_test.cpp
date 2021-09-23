#include <ros/ros.h>
#include <fstream>

#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/ConfigOptions.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <cartesian_interface/ros/RosServerClass.h>

#include "planner/multi_contact/Configuration.hpp"

using namespace XBot::Cartesian;

std::string env(std::getenv("ROBOTOLOGY_ROOT"));

int n_dof;

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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_stability_test_node");
    ros::NodeHandle nh("~");

    // Load ModelInterface from param server
    auto cfg = XBot::ConfigOptionsFromParamServer();
    auto model = XBot::ModelInterface::getModel(cfg);

    n_dof = model->getJointNum();

    // Read qList.txt
    std::vector<Configuration> qList;
    readFromFileConfigs(qList, "qList");

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

    auto ci = CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                   ik_prob, ci_ctx);

    rsc = std::make_shared<RosServerClass>(ci);

    // Take the first configuration from the qList
    Eigen::VectorXd q;
    q.segment(0,3) = qList[0].getFBPosition();
    q.segment(3,3) = qList[0].getFBOrientation();
    q.tail(n_dof-6) = qList[0].getJointValues();

    model->setJointPosition(q);
    model->update();

    return 0;
}
