#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/PointIndices.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>

#include "planner/multi_contact/utils.hpp" //PF

namespace XBot { namespace Planning {

class PointCloudManager {

public:

    //PointCloudManager(ros::NodeHandle nh, std::string topic_name);
   PointCloudManager ( ros::NodeHandle& nh, Eigen::Vector3d center, double side_x, double side_y, double side_z, double resolution );

    //void Callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
    
    void computeNormals(const double radius_search);
    
    Eigen::MatrixXd getPointCloud();
    
    Eigen::MatrixXd getNormals();
    
    bool callbackDone();
    
    void fromNormaltoMarkerArray(const double arrow_scale = 0.1);

    void refineNormals (const double radius_search);

    pcl::PointCloud< pcl::PointXYZ > getPCLPointCloud(); 


private:
    
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    ros::Publisher _pub;
    
    bool _callbackDone;
    Eigen::MatrixXd _pointCloud, _normals;
    
    //pcl::PointCloud<pcl::PointXYZ>::ConstPtr _pcl_pointcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pcl_pointcloud;
    pcl::PointCloud<pcl::Normal>::Ptr _pcl_normals;
};
} }