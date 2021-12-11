#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
//octree_change_detction
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <vector>
#include <ctime>

#include <pcl/point_types.h>

int main(int argc, char **argv)
{

    srand((unsigned int)time(NULL)); //随机种子

    //Octree resolution - side length of octree voxels
    float resolution = 15.0f;
    //实例化基于octree的点云检测类
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution); //创建检测类的对象
    //创建两个XYZ点云指针，想办法将文件中的数据赋值给该指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);//用来类型转换的
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>); //使用这个指针将获得的索引强制转换并输出

    //加载点云文件给cloud指针
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("room_scan1.pcd", *cloud1) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return (-1);
    }

    std::cout << "loaded room_scan1.pcd  " << cloud1->size() << std::endl;

    //改造原始点云
    pcl::PointCloud<pcl::PointXYZ> handledCloud(*cloud1); //拷贝复制

    //随机增加10000个点云,1024.f * rand() / (RAND_MAX + 1.0f)中的1024控制点云的范围
    for (int i = 0; i < 10000; i++)
    {
        handledCloud.push_back(
            pcl::PointXYZ(50.0f * rand() / (RAND_MAX + 1.0f), 50.0f * rand() / (RAND_MAX + 1.0f), 1.0f * rand() / (RAND_MAX + 1.0f)));
    }
    std::cout << "handledCloud points loaded  " << handledCloud.size() << endl;
    //pcl::io::loadPCDFile<pcl::PointXYZ>("room_scan2.pcd",*cloud2);
    //std::cout<<"loaded maize1"<<std::endl;

    //add cloud_point from cloud1 to octree
    octree.setInputCloud(cloud1);     //输入点云cloud1
    octree.addPointsFromInputCloud(); //从输入点云cloud1构建八叉树

    //Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree.switchBuffers(); //交换八叉树缓存，但是cloud1对应的八叉树结构仍在内存中

    //add points from cloud2 to octree
    octree.setInputCloud(handledCloud.makeShared()); //makeShared()返回一个指针
    octree.addPointsFromInputCloud();                //从输入点云cloud2构建八叉树

    std::vector<int> newPointIdxVector; //存储新加点的索引的向量

    //Get vector of point indices from octree voxels which did not exist in previous buffer
    octree.getPointIndicesFromNewVoxels(newPointIdxVector); //获取新增点的索引

    //将新增点的放到cloud_result所指向的内存中
    cloud_result->width = 50000;
    cloud_result->height = 1;
    cloud_result->is_dense = false;
    cloud_result->points.resize(cloud_result->width * cloud_result->height);
    for (size_t i = 0; i < newPointIdxVector.size(); ++i)
    {
        cloud_result->points[i].x = handledCloud.points[newPointIdxVector[i]].x;
        cloud_result->points[i].y = handledCloud.points[newPointIdxVector[i]].y;
        cloud_result->points[i].z = handledCloud.points[newPointIdxVector[i]].z;
    }

    //保存文件
    //pcl::io::savePCDFileASCII("result.pcd", *cloud_result);//将计算结果存放到result.pcd
    //std::cout << "result.pcd saved " << cloud_result->size() << "points" << endl;

    //不转换直接输出2
    std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
    std::cout << newPointIdxVector.size() << " points changed " << std::endl;
    /*for (size_t i = 0; i < newPointIdxVector.size(); ++i)
		std::cout << i << "# Index:" << newPointIdxVector[i]
		<< "  Point:" << handledCloud.points[newPointIdxVector[i]].x << " "
		<< handledCloud.points[newPointIdxVector[i]].y << " "
		<< handledCloud.points[newPointIdxVector[i]].z << std::endl;
	std::cout << "-----------------------------result end-------------------------" << std::endl;*/

    //指针赋值
    //pcl::io::loadPCDFile("result.pcd",*cloud3);//将新得到的result.pcd赋值给*cloud_3,读取

    //可视化模块
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewwe")); //创建指针viewer
    viewer->initCameraParameters();                                                                    //初始化相机参数

    int v1(0);                                                                   //第一个窗口的参数
    viewer->createViewPort(0.0, 0.0, 0.33, 1, v1);                               //设置第一个窗口的大小，位于屏幕左侧
    viewer->setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1); //background of first port
    viewer->addText("cloud1", 10, 10, "cloud1", v1);                             //好像是一个便签
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud1, 0, 255, 0);//设置第一个点云的颜色
    viewer->addPointCloud<pcl::PointXYZ>(cloud1, "simple_cloud1", v1); //显示第一个点云

    int v2(0);                                                                   //第儿个窗口的参数
    viewer->createViewPort(0.33, 0, 0.66, 1, v2);                                //设置第二个窗口的大小，位于屏幕右侧
    viewer->setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2); //background of second port
    viewer->addText("handledCloud", 10, 9, "handledCloud", v2);                  //输出一行文字
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(handledCloud.makeShared(), 255, 1, 1);//设置第二个点云的颜色
    viewer->addPointCloud<pcl::PointXYZ>(handledCloud.makeShared(), "simple_cloud2", v2); //显示第二个点云

    //第三个点云窗口
    int v3(0);                                                                   //第三个窗口的参数
    viewer->createViewPort(0.66, 0, 1, 1, v3);                                   //窗口大小
    viewer->setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v3); //背景颜色
    viewer->addText("cloud_result", 10, 8, "cloud_result", v3);                  //好像是一个便签
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cloud_result, 255, 255, 255);//点云颜色
    viewer->addPointCloud<pcl::PointXYZ>(cloud_result, "simple_cloud3", v3); //显示点云

    //viewer->createViewPort();
    viewer->addCoordinateSystem(2); //添加坐标系

    // while(!viewer->wasStopped())
    // viewer->spinOnce(100);
    viewer->spin();

    return 0;
}
