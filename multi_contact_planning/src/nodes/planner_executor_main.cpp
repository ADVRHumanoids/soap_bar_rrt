#include "planner_executor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_contact_planning_node");
    ros::NodeHandle nhpr("~");

    XBot::Cartesian::PlannerExecutor exec;

    ros::Rate rate(nhpr.param("rate", 30.));

    while(ros::ok())
    {
        exec.run();
        rate.sleep();
    }
}
