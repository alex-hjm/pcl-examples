#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conditional(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("room_scan1.pcd", *cloud);
    //radius_remove
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.5);        // 设定滤波半径
    outrem.setMinNeighborsInRadius(20); // 设定滤波半径范围内部的最低点云个数，小于次数的点被滤除
    outrem.filter(*cloud_radius);
    //pcl::io::savePCDFileASCII("cloud_radius.pcd", *cloud_radius);

    //condition_remove 的滤波器
    //build the filter
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>); //实例化条件指针
    // 为条件对象添加比较算子
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 1.0)));
    //build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true); //设置保持点云的结构，保存原有点云结结构就是点的数目没有减少，采用nan代替了
    //apply filter
    condrem.filter(*cloud_conditional);
    //pcl::io::savePCDFileASCII("cloud_conditional.pcd", *cloud_conditional);

    std::cerr << "cloud: " << cloud->size() << " points" << std::endl;
    std::cerr << "cloud_radius: " << cloud_radius->size() << " points" << std::endl;
    std::cerr << "cloud_conditional: " << cloud_conditional->size() << " points" << std::endl;
    //visualizer

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    viewer->initCameraParameters();

    int v1(0);
    viewer->createViewPort(0, 0, 0.33, 1, v1);
    viewer->setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud, 255, 0, 0);
    viewer->addText("Cloud ", 10, 10, "v2 test", v1);
    viewer->addPointCloud(cloud, "cloud_", v1); //C++赋值兼容规则。派生类对象可以用来初始化基类的引用

    int v2(0);
    viewer->createViewPort(0.33, 0, 0.66, 1, v2);
    viewer->setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud_radius, 255, 0, 0);
    viewer->addText("Cloud after radius_outlier_removal filtering", 10, 10, "v2 test", v2);
    viewer->addPointCloud(cloud_radius, "cloud_radius", v2);

    int v3(0);
    viewer->createViewPort(0.66, 0, 1, 1, v3);
    viewer->setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v3);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud_conditional, 255, 0, 0);
    viewer->addText("Cloud after conditional_removal filtering", 10, 10, "v2 test", v3);
    viewer->addPointCloud(cloud_conditional, "cloud_conditional", v3);

    viewer->addCoordinateSystem();
    viewer->spin();

    return 0;
}
