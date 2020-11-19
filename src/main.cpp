/*
*
*    author : Amith R
*
*/

#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointField.h>
#include <geometry_msgs/Point32.h>

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;
using namespace std;

// Viewer Routine
sensor_msgs::PointCloud cur_pointcloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>); // shared pointer to point cloud


pcl::visualization::PCLVisualizer::Ptr viewer;
bool pc(false);

void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-p            Point Cloud Visualisation\n"
            << "\n\n";
}

// create and initialise a 3D viewer
pcl::visualization::PCLVisualizer::Ptr createViewer(){
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer::Ptr ("3D Viewer"));
    viewer->setBackgroundColor(0 , 0 ,0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();
    return(viewer);
}

void updateViewer(pcl::visualization::PCLVisualizer::Ptr viewer , pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){

    //pcl::visualization::PointCloudColorHandlerRGBField rgb(cloud);
    viewer->removePointCloud();
    viewer->addPointCloud(cloud);
    viewer->spinOnce();

}

pcl::visualization::PCLVisualizer::Ptr Visualise(pcl::PointCloud<pcl::PointXYZ>::ConstPtr original_cloud){

    //Open 3D viewer and add the point cloud
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor(0 , 0 , 0);
    viewer->addPointCloud<pcl::PointXYZ>(original_cloud , "3d cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE , 1, "3d cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return viewer;
}


void PC2_callback(const sensor_msgs::PointCloud2& temp_pc){
   
    sensor_msgs::PointCloud point_cloud;
    
    bool result = sensor_msgs::convertPointCloud2ToPointCloud(temp_pc , point_cloud);

    cout << "Callback called " << endl;

    cur_pointcloud = point_cloud;

    //pcl conversion
    // check global variable reinitialisation
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>); // shared pointer to point cloud

    for(auto iter = cur_pointcloud.points.begin() ; iter!= cur_pointcloud.points.end() ; ++iter){
        pcl::PointXYZ p;
        p.x = (*iter).x; //(*iter) gets the contents of the iterator
        p.y = (*iter).y;
        p.z = (*iter).z;
        original_cloud->push_back(p);    
    }
      
}



int main( int argc  , char** argv){


    if(pcl::console::find_argument(argc , argv , "-h") >=0){
        printUsage(argv[0]);
        return 0;
    }
    

    if(pcl::console::find_argument(argc , argv , "-p") >= 0){
        pc = true;
        std::cout << "Point Cloud Visualisation \n" << std::endl;
    }
    else{
        printUsage(argv[0]);
        return 0;
    }

    ros::init(argc , argv , "PointCloud_Visualiser");
    ros::NodeHandle nh;

    //provide point cloud
    // Subscribe to sensormsgs::PointCloud and sensormsgs::PointCloud2 ROS Topics
    ros::Subscriber PC_subscriber , PC2_subscriber ;

    PC_subscriber = nh.subscribe("/camera/left/point_cloud2" , 1 , PC2_callback);

    // check global variable reintialisation
    if(pc){
        viewer = Visualise(original_cloud); 
    }

    ros::spin();
    return 0;
}
