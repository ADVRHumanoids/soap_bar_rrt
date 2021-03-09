#include <utils/point_cloud_manager.h>

using namespace XBot::Planning;

PointCloudManager::PointCloudManager ( ros::NodeHandle& nh ):
    _pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>),
    _pcl_normals(new pcl::PointCloud<pcl::Normal>),
    _nh(nh)
    
{
    Eigen::Vector3d center;
    double side_x, side_y, side_z;
    double resolution = RESOLUTION;
    int scenario = SCENARIO;
     
    double x, y, z;
    unsigned int index = 0;
    _pcl_pointcloud->resize(10000);
    _pub = nh.advertise<visualization_msgs::MarkerArray> ( "normals_plane", 1, true);
    
    /////////////////////////////////////////////////////////////////////////////////////////////////
    // STAND UP
    if(scenario == 1){
        center << 0.0, 0.0, 0.0;
        side_x = 5.0;
        side_y = 2.0;
        side_z = 3.0;
        for(int i = 1; i <= (int)(side_x/resolution)-1; i++)
        {
            x = center(0) - (side_x/2.0) + i*resolution;
            for(int j = 1; j <= (int)(side_y/resolution); j++)
            {
                y = center(1) - (side_y/2.0) + j*resolution;
                z = center(2) + 0.0;
                _pcl_pointcloud->points[index].x = x;
                _pcl_pointcloud->points[index].y = y;
                _pcl_pointcloud->points[index].z = z;
                index ++;
            }    
        }
        center << 2.5, 0.0, 1.5;
        for(int i = 1; i <= (int)(side_y/resolution); i++)
        {
            x = center(0);
            y = center(1) - (side_y/2.0) + i*resolution;
            for(int j = 1; j <= (int)(side_z/resolution); j++)
            {
                z = center(2) - (side_z/2.0) + j*resolution;
                _pcl_pointcloud->points[index].x = x;
                _pcl_pointcloud->points[index].y = y;
                _pcl_pointcloud->points[index].z = z;
                index ++;
            }
        }
    }
    // PARALLEL WALLS CLIMBING
    if(scenario == 2){
        center << 0.0, 0.0, 0.0;
        side_x = 2.0;
        side_y = 2.0;
        for(int i = 1; i <= (int)(side_x/resolution)-1; i++)
        {
            x = center(0) - (side_x/2.0) + i*resolution;
            for(int j = 1; j <= (int)(side_y/resolution); j++)
            {
                y = center(1) - (side_y/2.0) + j*resolution;
                z = center(2) + 0.0;
                _pcl_pointcloud->points[index].x = x;
                _pcl_pointcloud->points[index].y = y;
                _pcl_pointcloud->points[index].z = z;
                index ++;
            }    
        }
        center << 0.0, 0.0, 2.5;
        side_x = 2.0;
        side_z = 5.0;
        for(int i = 1; i <= (int)(side_x/resolution); i++)
        {
            x = center(0) - (side_x/2.0) + i*resolution;
            y = center(1) + 0.8;
            for(int j = 1; j <= (int)(side_z/resolution); j++)
            {
                z = center(2) - (side_z/2.0) + j*resolution;
                _pcl_pointcloud->points[index].x = x;
                _pcl_pointcloud->points[index].y = y;
                _pcl_pointcloud->points[index].z = z;
                index ++;
            }
        }
        for(int i = 1; i <= (int)(side_x/resolution); i++)
        {
            x = center(0) - (side_x/2.0) + i*resolution;
            y = center(1) - 0.8;
            for(int j = 1; j <= (int)(side_z/resolution); j++)
            {
                z = center(2) - (side_z/2.0) + j*resolution;
                _pcl_pointcloud->points[index].x = x;
                _pcl_pointcloud->points[index].y = y;
                _pcl_pointcloud->points[index].z = z;
                index ++;
            }
        }
    }
    // LADDER CLIMBING
    if(scenario == 3){
        center << 0.0, 0.0, 0.0;
        side_x = 2.0;
        side_y = 2.0;
        for(int i = 1; i <= (int)(side_x/resolution)-1; i++)
        {
            x = center(0) - (side_x/2.0) + i*resolution;
            for(int j = 1; j <= (int)(side_y/resolution); j++)
            {
                y = center(1) - (side_y/2.0) + j*resolution;
                z = center(2) + 0.0;
                _pcl_pointcloud->points[index].x = x;
                _pcl_pointcloud->points[index].y = y;
                _pcl_pointcloud->points[index].z = z;
                index ++;
            }    
        }
        side_x = 0.1;
        side_y = 1.0;
        int n_stair = 13;
        double x_displacement = 0.1;
        double z_displacement = 0.2;
        center << 1.0, 0.0, 0.2;
        for(int k = 0; k < n_stair; k++){
            if(k != n_stair-2 && k != n_stair-3){
                for(int i = 1; i <= (int)(side_x/resolution)-1; i++)
                {
                    x = center(0) - (side_x/2.0) + i*resolution;
                    z = center(2);
                    std::cout << "x = " << x << std::endl;
                    std::cout << "z = " << z << std::endl;
                    for(int j = 1; j <= (int)(side_y/resolution); j++)
                    {
                        y = center(1) - (side_y/2.0) + j*resolution;
                        _pcl_pointcloud->points[index].x = x;
                        _pcl_pointcloud->points[index].y = y;
                        _pcl_pointcloud->points[index].z = z;
                        index ++;
                    }    
                }
            }
            
            center(0) += x_displacement;
            center(2) += z_displacement;
        
        }
        
//         side_x = 0.1;
//         side_y = 0.2;
//         //center << 1.0 + n_stair*x_displacement, 0.6, 0.2 + n_stair*z_displacement;
//         center << 1.0 + n_stair*x_displacement + (side_x/2.0), 0.6, 0.2 + n_stair*z_displacement + 0.5;
//         for(int i = 1; i <= (int)(side_x/resolution)-1; i++)
//         {
//             x = center(0) - (side_x/2.0) + i*resolution;
//             z = center(2);
//             std::cout << "x = " << x << std::endl;
//             std::cout << "z = " << z << std::endl;
//             for(int j = 1; j <= (int)(side_y/resolution); j++)
//             {
//                 y = center(1) - (side_y/2.0) + j*resolution;
//                 _pcl_pointcloud->points[index].x = x;
//                 _pcl_pointcloud->points[index].y = y;
//                 _pcl_pointcloud->points[index].z = z;
//                 index ++;
//             }    
//         }
//         //center << 1.0 + n_stair*x_displacement, -0.6, 0.2 + n_stair*z_displacement;
//         center << 1.0 + n_stair*x_displacement + (side_x/2.0), -0.6, 0.2 + n_stair*z_displacement + 0.5;
//         for(int i = 1; i <= (int)(side_x/resolution)-1; i++)
//         {
//             x = center(0) - (side_x/2.0) + i*resolution;
//             z = center(2);
//             std::cout << "x = " << x << std::endl;
//             std::cout << "z = " << z << std::endl;
//             for(int j = 1; j <= (int)(side_y/resolution); j++)
//             {
//                 y = center(1) - (side_y/2.0) + j*resolution;
//                 _pcl_pointcloud->points[index].x = x;
//                 _pcl_pointcloud->points[index].y = y;
//                 _pcl_pointcloud->points[index].z = z;
//                 index ++;
//             }    
//         }
// 
//         side_x = 2.0;
//         side_y = 2.0;
//         center << 1.0 + n_stair*x_displacement + (side_x/2.0), 0.0, 0.2 + n_stair*z_displacement;
//         for(int i = 1; i <= (int)(side_x/resolution)-1; i++)
//         {
//             x = center(0) - (side_x/2.0) + i*resolution;
//             z = center(2);
//             std::cout << "x = " << x << std::endl;
//             std::cout << "z = " << z << std::endl;
//             for(int j = 1; j <= (int)(side_y/resolution); j++)
//             {
//                 y = center(1) - (side_y/2.0) + j*resolution;
//                 _pcl_pointcloud->points[index].x = x;
//                 _pcl_pointcloud->points[index].y = y;
//                 _pcl_pointcloud->points[index].z = z;
//                 index ++;
//             }    
//         }
//         
    }
    
    
   /////////////////////////////////////////////////////////////////////////////////////////////////
    
    _pointCloud.resize(_pcl_pointcloud->width, 3);
    for(int i = 0; i < _pcl_pointcloud->width; i++)
    {
        _pointCloud(i,0) = _pcl_pointcloud->points[i].x;
        _pointCloud(i,1) = _pcl_pointcloud->points[i].y;
        _pointCloud(i,2) = _pcl_pointcloud->points[i].z;
    }
    
    _pcl_pointcloud->header.frame_id = "world";
    
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
    
    ////////////////////////////////////////////////////////////////////////////////////////////////
    
    if(SCENARIO == 3){
        for ( int i = 0; i < _pcl_pointcloud->width; i++ ) {
            _pcl_normals->points[i].normal_x = 0.0;
            _pcl_normals->points[i].normal_y = 0.0;
            _pcl_normals->points[i].normal_z = 1.0;        
        }
    }
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

pcl::PointCloud< pcl::PointXYZ > PointCloudManager::getPCLPointCloud() 
{
    return *_pcl_pointcloud;
}




