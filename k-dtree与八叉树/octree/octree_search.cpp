#include <pcl/io/pcd_io.h>                  //文件输入输出
#include <pcl/octree/octree_search.h>       //octree相关定义
#include <pcl/visualization/cloud_viewer.h> //vtk可视化相关定义
#include <pcl/point_types.h>                //点类型相关定义

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

    //3.创建Octree实例对象
    float resolution = 0.03f;                                                 //设置octree体素分辨率
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution); //建立octree对象
    octree.setInputCloud(cloud);                                              //传入需要建立kdtree的点云指针
    octree.addPointsFromInputCloud();                                         //构建Octree

    //3.1.体素内近邻搜索
    pcl::PointXYZRGB searchPoint1 = cloud->points[1250]; //设置查找点
    std::vector<int> pointIdxVec;                        //保存体素近邻搜索的结果向量
    if (octree.voxelSearch(searchPoint1, pointIdxVec))
    {
        std::cout << "Neighbors within voxel(red) search at (" << searchPoint1.x
                  << " " << searchPoint1.y
                  << " " << searchPoint1.z << ")"
                  << std::endl;
        //for (size_t i = 0; i < pointIdxVec.size(); i++)
        //{
        //	std::cout << "   " << cloud->points[pointIdxVec[i]].x
        //		<< ", " << cloud->points[pointIdxVec[i]].y
        //		<< ", " << cloud->points[pointIdxVec[i]].z << std::endl;
        //}
        //给查找到的近邻点设置颜色
        for (size_t i = 0; i < pointIdxVec.size(); ++i)
        {
            cloud->points[pointIdxVec[i]].r = 255;
            cloud->points[pointIdxVec[i]].g = 0;
            cloud->points[pointIdxVec[i]].b = 0;
        }

        std::cout << "体素内近邻点个数：" << pointIdxVec.size() << endl;
    }

    //3.2.K近邻搜索
    pcl::PointXYZRGB searchPoint2 = cloud->points[3000]; //设置查找点
    int K = 200;
    std::vector<int> pointIdxNKNSearch;         //保存K近邻点的索引结果
    std::vector<float> pointNKNSquaredDistance; //保存每个近邻点与查找点之间的欧式距离平方

    std::cout << "K nearest neighbor(green) search at (" << searchPoint2.x
              << " " << searchPoint2.y
              << " " << searchPoint2.z
              << ") with K=" << K << std::endl;

    if (octree.nearestKSearch(searchPoint2, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        //for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
        //	std::cout << "  " << cloud->points[pointIdxNKNSearch[i]].x
        //	<< " " << cloud->points[pointIdxNKNSearch[i]].y
        //	<< " " << cloud->points[pointIdxNKNSearch[i]].z
        //	<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        //给查找到的近邻点设置颜色
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
        {
            cloud->points[pointIdxNKNSearch[i]].r = 0;
            cloud->points[pointIdxNKNSearch[i]].g = 255;
            cloud->points[pointIdxNKNSearch[i]].b = 0;
        }
    }
    std::cout << "K = 200近邻点个数：" << pointIdxNKNSearch.size() << endl;

    //3.3.半径内近邻搜索
    pcl::PointXYZRGB searchPoint3 = cloud->points[6500]; //设置查找点
    std::vector<int> pointIdxRadiusSearch;               //保存每个近邻点的索引
    std::vector<float> pointRadiusSquaredDistance;       //保存每个近邻点与查找点之间的欧式距离平方
    float radius = 0.02;                                 //设置查找半径范围

    std::cout << "Neighbors within radius(blue) search at (" << searchPoint3.x
              << " " << searchPoint3.y
              << " " << searchPoint3.z
              << ") with radius=" << radius << std::endl;

    if (octree.radiusSearch(searchPoint3, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        //for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        //	std::cout << "  " << cloud->points[pointIdxRadiusSearch[i]].x
        //	<< " " << cloud->points[pointIdxRadiusSearch[i]].y
        //	<< " " << cloud->points[pointIdxRadiusSearch[i]].z
        //	<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
        //给查找到的近邻点设置颜色
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
            cloud->points[pointIdxRadiusSearch[i]].r = 0;
            cloud->points[pointIdxRadiusSearch[i]].g = 0;
            cloud->points[pointIdxRadiusSearch[i]].b = 255;
        }
    }
    std::cout << "半径0.02近邻点个数： " << pointIdxRadiusSearch.size() << endl;

    //4.显示点云
    pcl::visualization::CloudViewer viewer("cloud viewer");
    viewer.showCloud(cloud);

    system("pause");
    return 0;
}
