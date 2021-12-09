#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h> //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>  //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv) //sensor_msgsros系统
{
    //sensor_msgs::PointCloud2::Ptr cloud_blob(new sensor_msgs::PointCloud2), cloud_filtered_blob(new sensor_msgs::PointCloud2);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    //建立四个点云
    //二进制格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_blob(new pcl::PointCloud<pcl::PointXYZ>);
    //ASCII格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    // 填入点云数据
    /*pcl::PCDReader reader;
	reader.read("table_scene_lms400.pcd", *cloud_blob);*/
    pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloud_blob);
    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
    // 创建滤波器对象:使用叶大小为1cm的下采样
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_blob);        //载入点云
    sor.setLeafSize(0.01f, 0.01f, 0.01f); //体素大大小为1cm
    sor.filter(*cloud_filtered);          //体素重心滤波

    // 转化为模板点云
    //pcl::fromROSMsg(*cloud_filtered_blob, *cloud_filtered);
    //二进制格式转化为ASCII格式
    //pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
    // 将下采样后的数据存入磁盘
    /*pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);*/

    //创建分割时所需要的模型系数对象coefficients
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    //创建存储内点的点索引对象inliers
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选
    seg.setOptimizeCoefficients(true); //设置对估计的模型参数进行优化处理
                                       //必须配置，设置分割的模型类型、所用的随机参数估计方法、距离阈值、输入点云
    // 必选
    seg.setModelType(pcl::SACMODEL_PLANE); //设置分割模型类别
    seg.setMethodType(pcl::SAC_RANSAC);    //设置参数估计的方法
    seg.setMaxIterations(1000);            //设置最大迭代次数
    seg.setDistanceThreshold(0.01);        //设置距离阈值
    // 创建滤波器对象
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    int i = 0, nr_points = (int)cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.3 * nr_points) // 当还有30%原始点云数据时
    {
        // 从余下的点云中分割最大平面组成部分
        seg.setInputCloud(cloud_filtered);    //载入待处理的点云
        seg.segment(*inliers, *coefficients); //分割操作
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // 分离内层
        extract.setInputCloud(cloud_filtered); // 载入点云
        extract.setIndices(inliers);           // 设置分割后的内点为需要提取的点集
        extract.setNegative(false);            // 设置提取内点
        extract.filter(*cloud_p);              // 开始分割
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
        std::stringstream ss;
        ss << "table_scene_lms400_plane_" << i << ".pcd";
        //writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);
        // 创建滤波器对象
        extract.setNegative(true); // 设置提取外点
        extract.filter(*cloud_f);  // 开始分割
        cloud_filtered.swap(cloud_f);
        i++;
    }

    // 可视化
    pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
    viewer.initCameraParameters();

    int v1(0);
    viewer.createViewPort(0.0, 0.0, 0.3, 1.0, v1);
    viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1); //背景色为冷灰色
    viewer.addText("Cloud before voxelgrid filtering", 10, 10, "v1 test", v1);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_blob, "cloud_blob", v1);

    int v2(0);
    viewer.createViewPort(0.3, 0.0, 0.6, 1.0, v2);
    viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
    viewer.addText("Cloud after voxelgrid filtering", 10, 10, "v2 test", v2);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, "cloud_filtered", v2);

    int v3(0);
    viewer.createViewPort(0.6, 0.0, 1.0, 1.0, v3);
    viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v3); //背景色为冷灰色
    viewer.addText("Cloud before voxelgrid filtering", 10, 10, "v3 test", v3);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_p, "cloud_p", v3);

    while (!viewer.wasStopped())
    {
        //使可视化窗口不会一闪而过
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return (0);
}
