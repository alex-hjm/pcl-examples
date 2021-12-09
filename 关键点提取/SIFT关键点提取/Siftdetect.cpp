#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
using namespace std;

namespace pcl
{
    template <>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float
        operator()(const PointXYZ &p) const
        {
            return p.z;
        }
    };
}

int main(int argc, char *argv[])
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(argv[1], *cloud_xyz); //将第二个命令行参数所代表的文件作为输入文件读入

    const float min_scale = atof(argv[2]);         //尺度空间中最小尺度的标准偏差
    const int n_octaves = atoi(argv[3]);           //高斯金字塔中组的数目
    const int n_scales_per_octave = atoi(argv[4]); //每组计算的尺度数目
    const float min_contrast = atof(argv[5]);      //设置关键点检测的阈值

    //SIFT关键点检测
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift; //创建sift关键点检测对象
    pcl::PointCloud<pcl::PointWithScale> result;                //SIFT关键点提取结果
    sift.setInputCloud(cloud_xyz);                              //设置输入点云
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    sift.setSearchMethod(tree);                                //创建一个空的kd树对象tree，并把它传递给sift检测对象
    sift.setScales(min_scale, n_octaves, n_scales_per_octave); //指定搜索关键点的尺度范围
    sift.setMinimumContrast(min_contrast);                     //设置限制关键点检测的阈值
    sift.compute(result);                                      //执行sift关键点检测，保存结果在result

    //类型转换
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result, *cloud_temp); //将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据

    //可视化输入点云和关键点
    pcl::visualization::PCLVisualizer viewer("Sift keypoint");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud(cloud_xyz, "cloud"); //在视窗中添加原始点云数据
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud");
    viewer.addPointCloud(cloud_temp, "keypoints"); //将SIFT关键点添加至视窗
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "keypoints");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}
