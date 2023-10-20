#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/compression/compression_profiles.h>
#include <pcl/compression/octree_pointcloud_compression.h>
 
int main(int argc, char *argv[])
{
    //创建3个点的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clound(new pcl::PointCloud<pcl::PointXYZRGB>());
    clound->width = 3;
    clound->height = 1;
    clound->resize(clound->width * clound->height);
    
    //将点云的点赋值为（-1, -1, -1）、（0.5, 0.5, 0.5）和（1.5, 1.5, 1.5）
    
    int index = 0;
    clound->points[index].x = -1;
    clound->points[index].y = -1;
    clound->points[index].z = -1;
    clound->points[index].r = 255;
    clound->points[index].g = 0;
    clound->points[index].b = 0;
 
 
    index = 1;
    clound->points[index].x = 0.5;
    clound->points[index].y = 0.5;
    clound->points[index].z = 0.5;
    clound->points[index].r = 255;
    clound->points[index].g = 0;
    clound->points[index].b = 0;
 
 
    index = 2;
    clound->points[index].x = 1.5;
    clound->points[index].y = 1.5;
    clound->points[index].z = 1.5;
    clound->points[index].r = 255;
    clound->points[index].g = 0;
    clound->points[index].b = 0;
    
    //显示点云
    for(int kk = 0; kk < clound->points.size(); ++kk)
    {
        std::cout << "Old cloud pt " << (kk + 1) << " : ( " << clound->points[kk].x << " , " << clound->points[kk].y << " , " << clound->points[kk].z << " )" << std::endl;
    }
    pcl::visualization::CloudViewer * viewer1 = new pcl::visualization::CloudViewer("ss1");
    pcl::visualization::CloudViewer * viewer2 = new pcl::visualization::CloudViewer("ss2");
    viewer1->showCloud(clound, "ss1");
    if(!viewer1->wasStopped())
    {
        //创建八叉树点云压缩对象
        pcl::io::compression_Profiles_e pro = pcl::io::MANUAL_CONFIGURATION;
        pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> * enc = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(pro, false, 0.01, 1, true);
        std::stringstream ss;
 
 
        //编码压缩
        enc->encodePointCloud(clound->makeShared(), ss);
        int octee_depth = enc->getTreeDepth();
        std::cout << "enc->getTreeDepth() : " << octee_depth << std::endl;
 
 
        //编码压缩后的点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clound_out(new pcl::PointCloud<pcl::PointXYZRGB>());
        enc->decodePointCloud(ss, clound_out);
        for(int kk = 0; kk < clound_out->points.size(); ++kk)
        {
            clound_out->points[kk].r = 0;
            clound_out->points[kk].g = 255;
            clound_out->points[kk].b = 0;
            std::cout << "New cloud pt " << (kk + 1) << " : ( " << clound_out->points[kk].x << " , " << clound_out->points[kk].y << " , " << clound_out->points[kk].z << " )" << std::endl;
        }
 
 
        //显示压缩后的点云
        viewer2->showCloud(clound_out, "ss2");
    }
    while(!viewer1->wasStopped() && !viewer2->wasStopped())
    {
    }
    delete viewer1,viewer2;
    viewer1 = nullptr;
    viewer2 = nullptr;

     return 0;
}