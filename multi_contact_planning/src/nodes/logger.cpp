#include <ros/ros.h>
#include <ros/subscriber.h>
#include <matlogger2/matlogger2.h>
#include <geometry_msgs/WrenchStamped.h>

XBot::MatLogger2::Ptr logger;
geometry_msgs::WrenchStamped reference_l_sole, reference_r_sole, reference_l_ball_tip, reference_r_ball_tip;
bool callbackDone = false;

void callback_value (const geometry_msgs::WrenchStampedConstPtr& msg)
{
    logger->add(msg->header.frame_id.substr(3) + "_value_fx", msg->wrench.force.x);
    logger->add(msg->header.frame_id.substr(3) + "_value_fy", msg->wrench.force.y);
    logger->add(msg->header.frame_id.substr(3) + "_value_fz", msg->wrench.force.z);
    logger->add(msg->header.frame_id.substr(3) + "_value_tx", msg->wrench.torque.x);
    logger->add(msg->header.frame_id.substr(3) + "_value_ty", msg->wrench.torque.y);
    logger->add(msg->header.frame_id.substr(3) + "_value_tz", msg->wrench.torque.z);

    if (callbackDone && msg->header.frame_id.substr(3) == "l_sole")
    {
        logger->add(msg->header.frame_id.substr(3) + "_reference_fx", reference_l_sole.wrench.force.x);
        logger->add(msg->header.frame_id.substr(3) + "_reference_fy", reference_l_sole.wrench.force.y);
        logger->add(msg->header.frame_id.substr(3) + "_reference_fz", reference_l_sole.wrench.force.z);
        logger->add(msg->header.frame_id.substr(3) + "_reference_tx", reference_l_sole.wrench.torque.x);
        logger->add(msg->header.frame_id.substr(3) + "_reference_ty", reference_l_sole.wrench.torque.y);
        logger->add(msg->header.frame_id.substr(3) + "_reference_tz", reference_l_sole.wrench.torque.z);
    }

    if (callbackDone && msg->header.frame_id.substr(3) == "r_sole")
    {
        logger->add(msg->header.frame_id.substr(3) + "_reference_fx", reference_r_sole.wrench.force.x);
        logger->add(msg->header.frame_id.substr(3) + "_reference_fy", reference_r_sole.wrench.force.y);
        logger->add(msg->header.frame_id.substr(3) + "_reference_fz", reference_r_sole.wrench.force.z);
        logger->add(msg->header.frame_id.substr(3) + "_reference_tx", reference_r_sole.wrench.torque.x);
        logger->add(msg->header.frame_id.substr(3) + "_reference_ty", reference_r_sole.wrench.torque.y);
        logger->add(msg->header.frame_id.substr(3) + "_reference_tz", reference_r_sole.wrench.torque.z);
    }

    if (callbackDone && msg->header.frame_id.substr(3) == "l_ball_tip")
    {
        logger->add(msg->header.frame_id.substr(3) + "_reference_fx", reference_l_ball_tip.wrench.force.x);
        logger->add(msg->header.frame_id.substr(3) + "_reference_fy", reference_l_ball_tip.wrench.force.y);
        logger->add(msg->header.frame_id.substr(3) + "_reference_fz", reference_l_ball_tip.wrench.force.z);
        logger->add(msg->header.frame_id.substr(3) + "_reference_tx", reference_l_ball_tip.wrench.torque.x);
        logger->add(msg->header.frame_id.substr(3) + "_reference_ty", reference_l_ball_tip.wrench.torque.y);
        logger->add(msg->header.frame_id.substr(3) + "_reference_tz", reference_l_ball_tip.wrench.torque.z);
    }

    if (callbackDone && msg->header.frame_id.substr(3) == "r_ball_tip")
    {
        logger->add(msg->header.frame_id.substr(3) + "_reference_fx", reference_r_ball_tip.wrench.force.x);
        logger->add(msg->header.frame_id.substr(3) + "_reference_fy", reference_r_ball_tip.wrench.force.y);
        logger->add(msg->header.frame_id.substr(3) + "_reference_fz", reference_r_ball_tip.wrench.force.z);
        logger->add(msg->header.frame_id.substr(3) + "_reference_tx", reference_r_ball_tip.wrench.torque.x);
        logger->add(msg->header.frame_id.substr(3) + "_reference_ty", reference_r_ball_tip.wrench.torque.y);
        logger->add(msg->header.frame_id.substr(3) + "_reference_tz", reference_r_ball_tip.wrench.torque.z);
    }

}

void callback_reference (const geometry_msgs::WrenchStampedConstPtr& msg)
{
    logger->add(msg->header.frame_id.substr(3) + "_reference_fx", msg->wrench.force.x);
    logger->add(msg->header.frame_id.substr(3) + "_reference_fy", msg->wrench.force.y);
    logger->add(msg->header.frame_id.substr(3) + "_reference_fz", msg->wrench.force.z);
    logger->add(msg->header.frame_id.substr(3) + "_reference_tx", msg->wrench.torque.x);
    logger->add(msg->header.frame_id.substr(3) + "_reference_ty", msg->wrench.torque.y);
    logger->add(msg->header.frame_id.substr(3) + "_reference_tz", msg->wrench.torque.z);

    if (msg->header.frame_id.substr(3) == "l_sole")
        reference_l_sole = *msg;
    else if (msg->header.frame_id.substr(3) == "r_sole")
        reference_r_sole = *msg;
    else if (msg->header.frame_id.substr(3) == "l_ball_tip")
        reference_l_ball_tip = *msg;
    else if (msg->header.frame_id.substr(3) == "r_ball_tip")
        reference_r_ball_tip = *msg;
    callbackDone = true;
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
        }
        else
        {
            sub_vect[i] = nh.subscribe<geometry_msgs::WrenchStamped>(argv[i+1], 10, callback_value);
        }
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
