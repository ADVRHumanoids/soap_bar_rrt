#include <ros/ros.h>
#include <ros/subscriber.h>
#include <matlogger2/matlogger2.h>
#include <geometry_msgs/WrenchStamped.h>

XBot::MatLogger2::Ptr logger;

void callback_value (const geometry_msgs::WrenchStampedConstPtr& msg)
{
    logger->add(msg->header.frame_id.substr(3) + "_value_fx", msg->wrench.force.x);
    logger->add(msg->header.frame_id.substr(3) + "_value_fy", msg->wrench.force.y);
    logger->add(msg->header.frame_id.substr(3) + "_value_fz", msg->wrench.force.z);
    logger->add(msg->header.frame_id.substr(3) + "_value_tx", msg->wrench.torque.x);
    logger->add(msg->header.frame_id.substr(3) + "_value_ty", msg->wrench.torque.y);
    logger->add(msg->header.frame_id.substr(3) + "_value_tz", msg->wrench.torque.z);
}

void callback_reference (const geometry_msgs::WrenchStampedConstPtr& msg)
{
    logger->add(msg->header.frame_id.substr(3) + "_reference_fx", msg->wrench.force.x);
    logger->add(msg->header.frame_id.substr(3) + "_reference_fy", msg->wrench.force.y);
    logger->add(msg->header.frame_id.substr(3) + "_reference_fz", msg->wrench.force.z);
    logger->add(msg->header.frame_id.substr(3) + "_reference_tx", msg->wrench.torque.x);
    logger->add(msg->header.frame_id.substr(3) + "_reference_ty", msg->wrench.torque.y);
    logger->add(msg->header.frame_id.substr(3) + "_reference_tz", msg->wrench.torque.z);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "logger_node");

    if (argc == 1)
        ROS_ERROR("[LOGGER]: please specify at least one topic name!");

    // Logger inizialization
    XBot::MatLogger2::Options opt;
    opt.default_buffer_size = 1e6;
    logger = XBot::MatLogger2::MakeLogger("/home/luca/MultiDoF-superbuild/external/soap_bar_rrt/multi_contact_planning/log/checks_log", opt);
    logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);


    // Variable inizialization
    ros::NodeHandle nh("");

    std::vector<ros::Subscriber> sub_vect;
    sub_vect.resize(argc - 1);

    // Subscribe to all the topics to be logged
    for (int i = 0; i < argc - 1; i++)
    {
        std::string topic_name(argv[i+1]);
        std::cout << topic_name << std::endl;


        if (topic_name.find("value") == std::string::npos)
        {
            sub_vect[i] = nh.subscribe<geometry_msgs::WrenchStamped>(argv[i+1], 10, callback_reference);
            std::cout << "reference" << std::endl;
        }
        else
        {
            sub_vect[i] = nh.subscribe<geometry_msgs::WrenchStamped>(argv[i+1], 10, callback_value);
            std::cout << "value" << std::endl;
        }
        std::getchar();
    }

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    logger.reset();
    return 0;
}
