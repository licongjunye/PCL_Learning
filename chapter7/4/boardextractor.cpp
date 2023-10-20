#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>


int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>());

    // 加载点云数据
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../1.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read the PCD file\n");
        return -1;
    }

    // 创建法线估计类的对象并设置其输入
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.02);
    ne.compute(*normals);

    // 创建边界提取对象并设置输入
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be;
    be.setInputCloud(cloud);
    be.setInputNormals(normals);
    be.setSearchMethod(tree);
    be.setRadiusSearch(0.02);
    be.compute(*boundaries);

    // 可视化边界
    pcl::visualization::PCLVisualizer viewer("Boundary Viewer");
    int v1 = 0;
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.setBackgroundColor(0, 0, 0, v1);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);
    int v2(0);
    viewer.createViewPort(0.5,0.0,1.0,1.0,v2);
    viewer.setBackgroundColor(0,0,0,v2);
    viewer.addPointCloud<pcl::PointXYZ>(cloud,"cloud2",v2);

    for (size_t i = 0; i < cloud->size(); i++) {
        if (boundaries->points[i].boundary_point == 1) {
            viewer.addSphere(cloud->points[i], 0.005, 1, 0, 0, "boundary_" + std::to_string(i), v1);  //给v1点云添加边缘
        }
    }

    viewer.spin();

    return 0;
}
