#include <utils/point_cloud_manager.h>

using namespace XBot::Planning;

PointCloudManager::PointCloudManager ( ros::NodeHandle nh, std::__cxx11::string topic_name ): 
    _nh(nh),
    _pcl_normals(new pcl::PointCloud<pcl::Normal>),
    _pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>)
    
{
    _sub = _nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(topic_name, 1000, &PointCloudManager::Callback, this);
    _pub = _nh.advertise<visualization_msgs::MarkerArray> ( "normals_plane", 1, true);
}

void PointCloudManager::Callback ( const pcl::PointCloud< pcl::PointXYZ >::ConstPtr& msg )
{
     
    _pcl_pointcloud = msg;
    _pointCloud.resize(msg->width, 3);        
    for(int i = 0; i < msg->width; i++)
    {
        _pointCloud(i,0) = msg->points[i].x;
        _pointCloud(i,1) = msg->points[i].y;
        _pointCloud(i,2) = msg->points[i].z;
    }
    _callbackDone = true;
    //std::cout << "pointcloud width: " << _pcl_pointcloud->width << std::endl;
    
}

void PointCloudManager::computeNormals (const double radius_search) 
{    
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud ( _pcl_pointcloud );

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree ( new pcl::search::KdTree<pcl::PointXYZ> );
    ne.setSearchMethod ( tree );

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch ( radius_search );

    // Compute normals
    ne.compute ( *_pcl_normals );
}

Eigen::MatrixXd PointCloudManager::getNormals()
{
    _normals.resize(_pcl_normals->width, 3);
    
    for (int i = 0; i < _pcl_normals->width; i++)
    {
        _normals(i,0) = _pcl_normals->points[i].normal_x;
        _normals(i,1) = _pcl_normals->points[i].normal_y;
        _normals(i,2) = _pcl_normals->points[i].normal_z;
    }
    
    return _normals;
}



Eigen::MatrixXd PointCloudManager::getPointCloud()
{
    return _pointCloud;
}

bool PointCloudManager::callbackDone() 
{
    return _callbackDone;
}

void PointCloudManager::fromNormaltoMarkerArray ( const double arrow_scale ) {

    // Dataset
    geometry_msgs::Point pnt_start, pnt_end;
    visualization_msgs::MarkerArray ma;


    ros::Time t = ros::Time::now();
    for ( int i = 0; i < _pcl_pointcloud->width; i++ ) {

        // Assign values to Marker fields
        if ( !std::isnan (_pcl_normals->points[i].normal_x ) && !std::isnan ( _pcl_normals->points[i].normal_y ) && !std::isnan ( _pcl_normals->points[i].normal_z ) ) {
            visualization_msgs::Marker m;

            m.header.frame_id = "world";
            m.header.stamp = t;
            m.id = i;
            m.action = visualization_msgs::Marker::ADD;
            m.type = visualization_msgs::Marker::ARROW;
            pnt_start.x = _pcl_pointcloud->points[i].x;
            pnt_start.y = _pcl_pointcloud->points[i].y;
            pnt_start.z = _pcl_pointcloud->points[i].z;
            m.points.push_back ( pnt_start );

            pnt_end.x = pnt_start.x + arrow_scale*_pcl_normals->points[i].normal_x;
            pnt_end.y = pnt_start.y + arrow_scale*_pcl_normals->points[i].normal_y;
            pnt_end.z = pnt_start.z + arrow_scale*_pcl_normals->points[i].normal_z;
            m.points.push_back ( pnt_end );
            m.scale.x = 0.01;
            m.scale.y = 0.02;
            m.scale.z = 0.02;
            m.color.r = 255;
            m.color.g = 0;
            m.color.b = 0;
            m.color.a = 1;

            // Assign Marker to MarkerArray
            ma.markers.push_back ( m );
        }
    }
    _pub.publish(ma);
}

void PointCloudManager::refineNormals (const double radius_search) 
{    
    int nullNormals = 0;
    Eigen::Vector3d p_i, p_j;

    for ( int i = 0; i < _pcl_pointcloud->width; i++ ) {
        bool validNormal = !std::isnan (_pcl_normals->points[i].normal_x ) && !std::isnan ( _pcl_normals->points[i].normal_y ) && !std::isnan ( _pcl_normals->points[i].normal_z) ;
    
        if ( !validNormal ) {

            /*
            p_i << pcl_pointcloud->points[i].x, _pcl_pointcloud->points[i].y, _pcl_pointcloud->points[i].z;
            int numNear = 0;
            Eigen::Vector3d normal_j;
            normal_j.setZero();

            for ( int j = 0; j < _pcl_pointcloud->width; j++ ) {
                if(i != j){
                    bool validNormal_j = !std::isnan (_pcl_normals->points[j].normal_x ) && !std::isnan ( _pcl_normals->points[j].normal_y ) && !std::isnan ( _pcl_normals->points[j].normal_z) ;
                    if(validNormal_j){
                        p_j << pcl_pointcloud->points[j].x, _pcl_pointcloud->points[j].y, _pcl_pointcloud->points[j].z;
                        double d = distance(p_i, p_j);
                        if(d < radius_search){
                            normal_j = normal_j + Eigen::Vector3d(_pcl_normals->points[j].normal_x, _pcl_normals->points[j].normal_y, _pcl_normals->points[j].normal_z);
                            numNear++;
                        }
                    }
                }
            }
            */

            p_i << _pcl_pointcloud->points[i].x, _pcl_pointcloud->points[i].y, _pcl_pointcloud->points[i].z;
        
            int jMin = -1;
            double dMin = 10000.0;    
            for ( int j = 0; j < _pcl_pointcloud->width; j++ ) {
                if(i != j){
                    bool validNormal_j = !std::isnan (_pcl_normals->points[j].normal_x ) && !std::isnan ( _pcl_normals->points[j].normal_y ) && !std::isnan ( _pcl_normals->points[j].normal_z) ;
                    if(validNormal_j){
                        p_j << _pcl_pointcloud->points[j].x, _pcl_pointcloud->points[j].y, _pcl_pointcloud->points[j].z;

                        if(abs(p_i(0) - p_j(0)) < 1e-3) jMin = j;
                        else if(abs(p_i(1) - p_j(1)) < 1e-3) jMin = j;
                        else if(abs(p_i(2) - p_j(2)) < 1e-3) jMin = j;         

                        /*    
                        double d = euclideanDistance(p_i, p_j);
                        if(d < dMin){
                            dMin = d;
                            jMin = j;
                        }
                        */
                    }
                }
            }
            if(jMin > -1){
                _pcl_normals->points[i].normal_x = _pcl_normals->points[jMin].normal_x;
                _pcl_normals->points[i].normal_y = _pcl_normals->points[jMin].normal_y;
                _pcl_normals->points[i].normal_z = _pcl_normals->points[jMin].normal_z;        
            }

            /*
            nullNormals++;

            _pcl_normals->points[i].normal_x = 0.0;
            _pcl_normals->points[i].normal_y = 0.0;
            _pcl_normals->points[i].normal_z = 1.0;
            */
        }
    }

    std::cout << "nullNormals = " << nullNormals << std::endl;
}




