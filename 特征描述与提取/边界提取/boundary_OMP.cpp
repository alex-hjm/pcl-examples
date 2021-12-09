#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/boundary.h>
#include <math.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/boundary.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/boundary.h>
#include <math.h>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>

int estimateBorders(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float re, float reforn)
{
    pcl::PointCloud<pcl::Boundary> boundaries;                                    //保存边界估计结果
    pcl::BoundaryEstimation<pcl::PointXYZI, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> normEst;                //定义一个法线估计的对象
    normEst.setNumberOfThreads(8);                                                // 手动设置线程数，否则提示错误
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);  //保存法线估计的结果
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZI>);
    normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr(cloud));
    normEst.setRadiusSearch(reforn); //设置法线估计的半径//normEst.setKSearch(10);//表示计算点云法向量时，搜索的点云个数
    normEst.compute(*normals);       //将法线估计结果保存至normals
                                     //输出法线的个数
std:
    cout << "reforn: " << reforn << std::endl;
    std::cerr << "normals: " << normals->size() << std::endl;

    boundEst.setInputCloud(cloud);                                                                               //设置输入的点云
    boundEst.setInputNormals(normals);                                                                           //设置边界估计的法线，因为边界估计依赖于法线
    boundEst.setRadiusSearch(re);                                                                                //设置边界估计所需要的半径,//这里的Threadshold为一个浮点值，可取点云模型密度的10倍
    boundEst.setAngleThreshold(M_PI / 4);                                                                        //边界估计时的角度阈值M_PI / 4  并计算k邻域点的法线夹角,若大于阈值则为边界特征点
    boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>)); //设置搜索方式KdTree
    boundEst.compute(boundaries);                                                                                //将边界估计结果保存在boundaries

    std::cerr << "AngleThreshold: " << M_PI / 4 << std::endl;
    //输出边界点的个数
    std::cerr << "boundaries: " << boundaries.points.size() << std::endl;
    //存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if (boundaries[i].boundary_point > 0)
        {
            cloud_boundary->push_back(cloud->points[i]);
        }
    }
    //pcl::PCDWriter writer;
    //std::stringstream ss;
    //ss << "boundary" << ".pcd";
    //writer.write<pcl::PointXYZI>(ss.str(), *cloud_boundary, false);
    //可视化显示原始点云与边界提取结果
    boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("边界提取"));

    int v1(0);
    MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
    MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
    int v2(0);
    MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
    MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
    MView->addText("Boudary point clouds", 80, 80, "v2_text", v2);
    MView->addPointCloud<pcl::PointXYZI>(cloud, "sample cloud", v1);
    MView->addPointCloud<pcl::PointXYZI>(cloud_boundary, "cloud_boundary", v2);
    MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
    MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_boundary", v2);
    MView->addCoordinateSystem(1.0);
    MView->initCameraParameters();
    MView->spin();
    return 0;
}
int main(int argc, char **argv)
{
    srand(time(NULL));
    float re, reforn;
    re = std::atof(argv[2]);
    reforn = std::atof(argv[3]);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(argv[1], *cloud_src);
    //创建滤波器对象
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud_src);
    sor.setMeanK(100);           //寻找每个点的50个最近邻点
    sor.setStddevMulThresh(3.0); //一个点的最近邻距离超过全局平均距离的一个标准差以上，就会舍弃
    sor.filter(*cloud_filtered);
    std::cout << "cloud_src: " << cloud_src->points.size() << std::endl;
    std::cout << "cloud_filtered: " << cloud_filtered->points.size() << std::endl;
    estimateBorders(cloud_src, re, reforn);
    return 0;
}
