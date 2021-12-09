#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>

int main()
{
    //加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //体素滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filterd(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloud);

    //体素滤波
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(*cloud_filterd);
    std::cout << "voxel filter succeed!" << std::endl;
    std::cerr << "PointCloud after filtering: " << cloud_filterd->size() << " data points (" << pcl::getFieldsList(*cloud_filterd) << " )" << std::endl; //getFieldsList(*cloud_filtered) 获取点的field(类似于维度）

    //估计法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; //实例化一个法线估计的对象
    ne.setInputCloud(cloud);                              //设置输入点云
    //创建一个空的kdtree对象，并把它传递给法线估计对象
    //基于给出的输入数据集，kdtree将被建立
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree); //设置近邻搜索方式
    //输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    //使用半径在查询点周围3厘米范围内的所有邻元素
    ne.setRadiusSearch(0.01); //如果曲率很重要，则减少这个尺度
    //计算特征值
    ne.compute(*cloud_normals);                                            //法线存放至cloud_normals
    std::cout << "normals: " << cloud_normals->points.size() << std::endl; //法向量的个数
    // cloud_normals->points.size ()应该与input cloud_downsampled->points.size ()有相同尺寸
    //法线可视化
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    int v1(0), v2(0);
    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPort(0.5, 0, 1, 1, v2);
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v1);
    viewer.setBackgroundColor(0, 20, 0, v2);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 100, 0.02, "cloud_mormals", v1);
    //viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals,v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singe_color2(cloud_filterd, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filterd, singe_color2, "cloud_filterd", v2);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}
