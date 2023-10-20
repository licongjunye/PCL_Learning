#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/make_shared.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

// #define FOLDERPATH "/home/hlc/livox_mid70_ws/map/map_daohang/"
#define FOLDERPATH "/home/hlc/livox_mid70_ws/map/maps_tree"

int
main(int argc,char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

#if 0
    //pcl方式打开点云文件
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("../test_pcd.pcd",*cloud)==-1)//打开点云文件
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
#else
    pcl::PCLPointCloud2 pcl_pc1;
    sensor_msgs::PointCloud2 ros_pc1;

    pcl::PCLPointCloud2 pcl_pc2;
    sensor_msgs::PointCloud2 ros_pc2;

    pcl::PCLPointCloud2 pcl_pc3;
    sensor_msgs::PointCloud2 ros_pc3;

    std::string pcdFile1 = std::string(FOLDERPATH) + "/all_points.pcd";
    std::string pcdFile2 = std::string(FOLDERPATH) + "/corner.pcd";
    std::string pcdFile3 = std::string(FOLDERPATH) + "/surf.pcd";

    // 1. 从PCD文件中读取到pcl::PCLPointCloud2类型
    if (pcl::io::loadPCDFile(pcdFile1, pcl_pc1) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return (-1);
    }

    if (pcl::io::loadPCDFile(pcdFile2, pcl_pc2) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return (-1);
    }

    if (pcl::io::loadPCDFile(pcdFile3, pcl_pc3) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return (-1);
    }

    // 2. 转换为ROS的PointCloud2类型
    pcl_conversions::fromPCL(pcl_pc1, ros_pc1);
    // 转换为pcl::PointCloud类型
    pcl::fromROSMsg(ros_pc1, *cloud1);

    // 2. 转换为ROS的PointCloud2类型
    pcl_conversions::fromPCL(pcl_pc2, ros_pc2);
    // 转换为pcl::PointCloud类型
    pcl::fromROSMsg(ros_pc2, *cloud2);

    // 2. 转换为ROS的PointCloud2类型
    pcl_conversions::fromPCL(pcl_pc3, ros_pc3);
    // 转换为pcl::PointCloud类型
    pcl::fromROSMsg(ros_pc3, *cloud3);
#endif


    std::cout<<"Loaded "
    <<cloud1->width*cloud1->height
    <<" data points from test_pcd.pcd with the following fields: "
    <<std::endl;
    for(size_t i=0;i<cloud1->points.size();++i){
        std::cout<<"    "<<cloud1->points[i].x
        <<" "<<cloud1->points[i].y
        <<" "<<cloud1->points[i].z<<std::endl;
    }

    //visualize
    boost::shared_ptr<pcl::visualization::PCLVisualizer> for_visualizer_v (new pcl::visualization::PCLVisualizer ("crophull display"));
    for_visualizer_v->setBackgroundColor(0,0,0);

    int v1(0);
    for_visualizer_v->createViewPort (0.0, 0.0, 0.33, 1, v1);
    for_visualizer_v->setBackgroundColor (0, 0, 0, v1);
    for_visualizer_v->addPointCloud (cloud1,"cloud1",v1);
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,255,255,"cloud1");
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,0.1,"cloud1");
    // for_visualizer_v->addPolygon<pcl::PointXYZ>(cloud1,0,.069*255,0.2*255,"backview_hull_polyline1",v1);

    int v2(0);
    for_visualizer_v->createViewPort (0.33, 0.0, 0.66, 1, v2);  
    for_visualizer_v->setBackgroundColor (0, 0, 0, v2);
    for_visualizer_v->addPointCloud (cloud2,"surface_hull",v2);
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,255,255,"surface_hull");
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,0.1,"surface_hull");
    // for_visualizer_v->addPolygon<pcl::PointXYZ>(cloud2,0,.069*255,0.2*255,"backview_hull_polyline",v2);

    int v3(0);
    for_visualizer_v->createViewPort (0.66, 0.0, 1, 1, v3);
    for_visualizer_v->setBackgroundColor (0, 0, 0, v3);
    for_visualizer_v->addPointCloud (cloud3,"objects",v3);
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,255,255,"objects");
    for_visualizer_v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,0.1,"objects");

    while (!for_visualizer_v->wasStopped())
    {

        for_visualizer_v->spinOnce(1000);
    }

    return(0);
}
