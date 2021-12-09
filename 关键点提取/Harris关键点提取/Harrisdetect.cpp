#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/keypoints/harris_3d.h> //harris特征点估计类头文件声明
#include <cstdlib>
#include <vector>
#include <pcl/console/parse.h>
using namespace std;

int main(int argc, char *argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>); //输入点云的指针
    pcl::io::loadPCDFile(argv[1], *input_cloud);                                         //将命令行参数文件读至input_cloud指针
    pcl::PCDWriter writer;
    float r_normal;   //法向量估计的半径
    float r_keypoint; //关键点估计的近邻搜索半径

    r_normal = atof(argv[2]);   //将命令行的第三个参数解析为法向量估计的半径
    r_keypoint = atof(argv[3]); //将命令行的第四个参数解析为关键点估计的近邻搜索半径

    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ColorHandlerT3;

    pcl::PointCloud<pcl::PointXYZI>::Ptr Harris_keypoints(new pcl::PointCloud<pcl::PointXYZI>()); //存放最后的特征点提取结果
    //实例化一个Harris特征检测对象harris_detector
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> *harris_detector = new pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>;

    //harris_detector->setNonMaxSupression(true);
    harris_detector->setRadius(r_normal);         //设置法向量估计的半径
    harris_detector->setRadiusSearch(r_keypoint); //设置关键点估计的近邻搜索半径
    harris_detector->setInputCloud(input_cloud);  //设置输入点云
    //harris_detector->setNormals(normal_source);
    //harris_detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::LOWE);
    harris_detector->compute(*Harris_keypoints); //结果存放在Harris_keypoints
    cout << "Harris_keypoints number: " << Harris_keypoints->size() << endl;
    //writer.write<pcl::PointXYZI> ("Harris_keypoints.pcd",*Harris_keypoints,false);//结果保存
    //pcl::io::savePCDFileASCII("Harris keypoints.pcd" , *Harris_keypoints);
    //cout<<"Points: "<<Harris_keypoints->points.size()<<endl;

    //可视化点云
    pcl::visualization::PCLVisualizer visu3("clouds");
    visu3.setBackgroundColor(255, 255, 255);
    //Harris_keypoints关键点可视化
    visu3.addPointCloud(Harris_keypoints, ColorHandlerT3(Harris_keypoints, 0.0, 0.0, 255.0), "Harris_keypoints");
    visu3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "Harris_keypoints");
    //原始点云可视化
    visu3.addPointCloud(input_cloud, "input_cloud");
    visu3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "input_cloud");
    visu3.spin();
}
