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

#include "planner/multi_contact/Configuration.hpp"

using namespace XBot::Cartesian;

std::string env(std::getenv("ROBOTOLOGY_ROOT"));

int n_dof;
int ind = 0;
std::vector<Configuration> qList;
Eigen::VectorXd q;

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

bool q_change_service(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
    ind++;
    if (ind == qList.size())
        ind = 0;

    Configuration c = qList[ind];
    q.segment(0,3) = c.getFBPosition();
    q.segment(3,3) = c.getFBOrientation();
    q.tail(n_dof-6) = c.getJointValues();

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_stability_test_node");
    ros::NodeHandle nh("~");
    ros::ServiceServer q_srv = nh.advertiseService("change_configuration", &q_change_service);

    // Load ModelInterface from param server
    auto cfg = XBot::ConfigOptionsFromParamServer();
    auto model = XBot::ModelInterface::getModel(cfg);

    n_dof = model->getJointNum();

    // Read qList.txt
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
    q.resize(n_dof);
    q.segment(0,3) = qList[ind].getFBPosition();
    q.segment(3,3) = qList[ind].getFBOrientation();
    q.tail(n_dof-6) = qList[ind].getJointValues();

    model->setJointPosition(q);
    model->update();

    return 0;
}
