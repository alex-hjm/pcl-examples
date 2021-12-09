#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>                           //��ʾ��
#include <pcl/visualization/cloud_viewer.h>                             //����ʾ����
#include <pcl/console/parse.h>                                          //pcl����̨����
#include <pcl/features/fpfh_omp.h>                                      //����fpfh���ټ����omp(��˲��м���)
#include <pcl/registration/correspondence_rejection_features.h>         //�����Ĵ����Ӧ��ϵȥ��
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //�������һ����ȥ��
#include <pcl/registration/icp.h>                                       //icp��׼
#include <pcl/features/boundary.h>                                      //�߽���ȡ
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h> //#include <boost/thread/thread.hpp>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/integral_image_normal.h>
#include <iostream>
// �������ͷ�ļ�

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
        std::cout << "û��help." << std::endl;
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
    //���ػ�
    pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_grid;
    approximate_voxel_grid.setLeafSize(0.5, 0.5, 0.5); //����߳�.�������ֵԽ���򾫼��Խ������ʣ�µ������٣�
    approximate_voxel_grid.setInputCloud(cloud);
    approximate_voxel_grid.filter(*source);
    cout << "voxel grid  Filte cloud size is: " << source->size() << endl;
    pcl::copyPointCloud(*source, *cloud);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Boundary> boundaries;
    //����߽���ȡ
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    /*
   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //����һ������k���ڲ�ѯ,��ѯ��ʱ�����õ��ڵ����У����һ�����ڵ����䱾��
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

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //����pcl::PointXYZ��ʾ�����������ݣ�pcl::Normal��ʾ�������,��pcl::Normalǰ�����Ƿ������һ��������
    normEst.setInputCloud(cloud);
    normEst.setSearchMethod(tree);
    //normEst.setRadiusSearch(2);  //������Ƶİ뾶
    normEst.setKSearch(20); //������Ƶĵ���
    normEst.compute(*normals);
    cout << "normal size is " << normals->size() << endl;

    //normal_est.setViewPoint(0,0,0); //���Ӧ�û�ʹ����һ��
    est.setInputCloud(cloud);
    est.setInputNormals(normals);
    //est.setAngleThreshold(90);
    //est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    est.setSearchMethod(tree);
    est.setKSearch(50); //һ���������ֵԽ�ߣ����ձ߽�ʶ��ľ���Խ��
    //est.setRadiusSearch(everagedistance);  //�����뾶
    est.compute(boundaries);
    cout << "boundaries size is " << boundaries.size() << endl;

    //�߽��
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
    int ppp = 0;
    for (int i = 0; i < cloud->size(); i++)
    {
        uint8_t x = (boundaries.points[i].boundary_point);

        int a = static_cast<int>(x); //�ú����Ĺ�����ǿ������ת��
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
    std::cout << "boudary size is��" << boundPoints->size() << std::endl;
    std::cout << "noBoundPoints size is��" << noBoundPoints.size() << std::endl;
    //pcl::io::savePCDFileASCII("boudary.pcd",boundPoints);

    //pcl::io::savePCDFileASCII("boudary.pcd", *boundPoints);
    //pcl::io::savePCDFileASCII("NoBoundpoints.pcd", noBoundPoints);
    //���ӻ���ʾԭʼ������߽���ȡ���
    boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("�߽���ȡ"));

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
    //MView->addCoordinateSystem(1.0); //ȡ������ϵ
    //MView->initCameraParameters();   //���������������ʹ�û���Ĭ�ϵĽǶȺͷ���۲����

    MView->spin();

    return 0;
}
