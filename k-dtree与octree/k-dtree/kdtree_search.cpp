#include <pcl/kdtree/kdtree_flann.h>        //kdtree近邻搜索
#include <pcl/io/pcd_io.h>                  //文件输入输出
#include <pcl/point_types.h>                //点类型相关定义
#include <pcl/visualization/cloud_viewer.h> //可视化相关定义
#include <iostream>
#include <vector>

int main()
{
    //1.读取点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("rabbit.pcd", *cloud) == -1)
    {
        PCL_ERROR("Cloudn't read file!");
        return -1;
    }

    //2.原始点云着色
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].r = 255;
        cloud->points[i].g = 255;
        cloud->points[i].b = 255;
    }

    //3.建立kd-tree
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree; //建立kdtree对象
    kdtree.setInputCloud(cloud);               //设置需要建立kdtree的点云指针

    //4.K近邻搜索
    pcl::PointXYZRGB searchPoint = cloud->points[1000]; //设置查找点
    int K = 900;                                        //设置需要查找的近邻点个数
    std::vector<int> pointIdxNKNSearch(K);              //保存每个近邻点的索引
    std::vector<float> pointNKNSquaredDistance(K);      //保存每个近邻点与查找点之间的欧式距离平方

    std::cout << "K nearest neighbor(green) search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;

    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
        {
            cloud->points[pointIdxNKNSearch[i]].r = 0;
            cloud->points[pointIdxNKNSearch[i]].g = 255;
            cloud->points[pointIdxNKNSearch[i]].b = 0;
        }
    }
    std::cout << "K = 900近邻点个数：" << pointIdxNKNSearch.size() << endl;

    //5.radius半径搜索
    pcl::PointXYZRGB searchPoint1 = cloud->points[3500]; //设置查找点
    std::vector<int> pointIdxRadiusSearch;               //保存每个近邻点的索引
    std::vector<float> pointRadiusSquaredDistance;       //保存每个近邻点与查找点之间的欧式距离平方
    float radius = 0.03;                                 //设置查找半径范围

    std::cout << "Neighbors within radius(red) search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with radius=" << radius << std::endl;

    if (kdtree.radiusSearch(searchPoint1, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
            cloud->points[pointIdxRadiusSearch[i]].r = 255;
            cloud->points[pointIdxRadiusSearch[i]].g = 0;
            cloud->points[pointIdxRadiusSearch[i]].b = 0;
        }
    }
    std::cout << "半径0.03近邻点个数： " << pointIdxRadiusSearch.size() << endl;

    //6.显示点云
    pcl::visualization::CloudViewer viewer("cloud viewer");
    viewer.showCloud(cloud);

    system("pause");
    return 0;
}
