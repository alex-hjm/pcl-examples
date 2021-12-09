#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

//#include <ctime>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>         //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/integral_image_normal.h>
#include <iostream>

#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include <pcl/common/common.h>
// 包含相关头文件

// This function displays the help
void showHelp(char *program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h or --help :  Show  help." << std::endl;
}

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointNormal PointNT; // 也可以pcl::Normal,但无法用PCLVisualizer显示。
//typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<PointNT> pointnormal;

// This is the main function
int main(int argc, char **argv)
{
    pcl::console::TicToc time;
    time.tic();
    // Show help
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        std::cout << "没有help." << std::endl;
        return 0;
    }

    // Fetch point cloud filename in arguments | Works with PCD and PLY files
    std::vector<int> filenames;
    bool file_is_pcd = false;

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

    if (filenames.size() != 1)
    {
        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

        if (filenames.size() != 1)
        {
            showHelp(argv[0]);
            return -1;
        }
        else
        {
            file_is_pcd = true;
        }
    }
    // Load file | Works with PCD and PLY files
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (file_is_pcd)
    {
        if (pcl::io::loadPCDFile(argv[filenames[0]], *cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                      << std::endl;
            showHelp(argv[0]);
            return -1;
        }
    }
    else
    {
        if (pcl::io::loadPLYFile(argv[filenames[0]], *cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                      << std::endl;
            showHelp(argv[0]);
            return -1;
        }
    }
    std::cout << "点云个数: " << cloud->size() << std::endl
              << std::endl;

    //法向量
    pointnormal::Ptr normals(new pointnormal);
    //pcl::NormalEstimation<pcl::PointXYZ, PointNT> est_normal;//4.2s
    pcl::NormalEstimationOMP<pcl::PointXYZ, PointNT> est_normal; //1.5s

    est_normal.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    est_normal.setSearchMethod(tree);
    // Use all neighbors in a sphere of radius 5cm
    est_normal.setKSearch(20);
    //est_normal.setRadiusSearch(2);  //法向估计的半径
    est_normal.compute(*normals);
    // cloud_normals->size () should have the same size as the input cloud->size ()*
    //计算单点法线
    //computePointNormal(const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices, Eigen::Vector4f & plane_parameters, float& curvature);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    { // 生成时只生成了法向量，没有将原始点云信息拷贝，为了显示需要复制原信息
        // 也可用其他方法进行连接，如：pcl::concatenateFields
        normals->points[i].x = cloud->points[i].x;
        normals->points[i].y = cloud->points[i].y;
        normals->points[i].z = cloud->points[i].z;
    }

    // 定义对象 （""）里是名称
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    // Setting visualiser window position窗口弹出位置
    viewer.setPosition(300, 100);
    // 2.Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, 0, 255, 0);
    viewer.addPointCloud(cloud, cloud_color, "original_cloud");
    //坐标系的三维轴添加到屏幕的(0, 0, 0)处
    //viewer.addCoordinateSystem(1.0, "cloud1", 0);
    // Setting background to a dark grey设置背景颜色，默认黑色
    viewer.setBackgroundColor(0.1, 0.1, 0.1);
    //渲染属性
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
    //显示法线
    int level = 1;      // 多少条法向量集合显示成一条
    float scale = 0.01; // 法向量长度
    viewer.addPointCloudNormals<pcl::PointXYZ, PointNT>(cloud, normals, level, scale);

    cout << "代码段运行时间: " << time.toc() / 1000 << "s" << endl; //计算程序运行时间
    while (!viewer.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }

    //system("pause");//不注释，命令行会提示按任意键继续，注释会直接跳出
    return 0;
}
