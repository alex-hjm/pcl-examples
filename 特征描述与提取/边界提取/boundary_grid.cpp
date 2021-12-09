#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>                           //显示库
#include <pcl/visualization/cloud_viewer.h>                             //简单显示点云
#include <pcl/console/parse.h>                                          //pcl控制台解析
#include <pcl/features/fpfh_omp.h>                                      //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_rejection_features.h>         //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/registration/icp.h>                                       //icp配准
#include <pcl/features/boundary.h>                                      //边界提取
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h> //#include <boost/thread/thread.hpp>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/integral_image_normal.h>
#include <iostream>
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
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
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
    std::cout << "\ncloud size is: " << cloud->size() << std::endl;
    pcl::copyPointCloud(*cloud, *cloud1);
    //体素化
    pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_grid;
    approximate_voxel_grid.setLeafSize(0.5, 0.5, 0.5); //网格边长.这里的数值越大，则精简的越厉害（剩下的数据少）
    approximate_voxel_grid.setInputCloud(cloud);
    approximate_voxel_grid.filter(*source);
    cout << "voxel grid  Filte cloud size is: " << source->size() << endl;
    pcl::copyPointCloud(*source, *cloud);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Boundary> boundaries;
    //定义边界提取
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    /*
   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
   kdtree.setInputCloud(cloud);
   int k =2;
   float everagedistance =0;
   for (int i =0; i < cloud->size()/2;i++)
   {
           vector<int> nnh ;
           vector<float> squaredistance;
           //  pcl::PointXYZ p;
           //   p = cloud->points[i];
           kdtree.nearestKSearch(cloud->points[i],k,nnh,squaredistance);
           everagedistance += sqrt(squaredistance[1]);
           //   cout<<everagedistance<<endl;
   }

   everagedistance = everagedistance/(cloud->size()/2);
   cout<<"everage distance is : "<<everagedistance<<endl;

*/

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
    normEst.setInputCloud(cloud);
    normEst.setSearchMethod(tree);
    //normEst.setRadiusSearch(2);  //法向估计的半径
    normEst.setKSearch(20); //法向估计的点数
    normEst.compute(*normals);
    cout << "normal size is " << normals->size() << endl;

    //normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
    est.setInputCloud(cloud);
    est.setInputNormals(normals);
    //est.setAngleThreshold(90);
    //est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    est.setSearchMethod(tree);
    est.setKSearch(50); //一般这里的数值越高，最终边界识别的精度越好
    //est.setRadiusSearch(everagedistance);  //搜索半径
    est.compute(boundaries);
    cout << "boundaries size is " << boundaries.size() << endl;

    //边界点
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
    int ppp = 0;
    for (int i = 0; i < cloud->size(); i++)
    {
        uint8_t x = (boundaries.points[i].boundary_point);

        int a = static_cast<int>(x); //该函数的功能是强制类型转换
        if (a == 1)
        {
            boundPoints->push_back(cloud->points[i]);
            ppp = i;
        }
        else
        {
            noBoundPoints.push_back(cloud->points[i]);
        }
    }
    std::cout << "boundaries.points[i].boundary_point:" << boundaries.points[ppp].boundary_point << std::endl;
    std::cout << "boudary size is：" << boundPoints->size() << std::endl;
    std::cout << "noBoundPoints size is：" << noBoundPoints.size() << std::endl;
    //pcl::io::savePCDFileASCII("boudary.pcd",boundPoints);

    //pcl::io::savePCDFileASCII("boudary.pcd", *boundPoints);
    //pcl::io::savePCDFileASCII("NoBoundpoints.pcd", noBoundPoints);
    //可视化显示原始点云与边界提取结果
    boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("边界提取"));

    int v1(0);
    MView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    MView->setBackgroundColor(0.3, 0.3, 0.3, v1);
    MView->addPointCloud<pcl::PointXYZ>(cloud1, "sample cloud", v1);
    MView->addPointCloud<pcl::PointXYZ>(boundPoints, "cloud_boundary1", v1);
    MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
    MView->addText("Raw point clouds", 10, 10, "v1_text", v1);
    int v2(0);
    MView->createViewPort(0.5, 0.0, 1, 1.0, v2);
    MView->setBackgroundColor(0.5, 0.5, 0.5, v2);
    MView->addText("Boudary point clouds", 80, 80, "v2_text", v2);

    MView->addPointCloud<pcl::PointXYZ>(boundPoints, "cloud_boundary", v2);
    MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_boundary", v2);
    //MView->addCoordinateSystem(1.0); //取消坐标系
    //MView->initCameraParameters();   //设置照相机参数，使用户从默认的角度和方向观察点云

    MView->spin();

    return 0;
}
