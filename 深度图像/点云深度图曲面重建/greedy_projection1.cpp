#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/print.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/console/time.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>
#include <pcl/surface/impl/organized_fast_mesh.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/visualization/common/float_image_utils.h> //保存深度图像
#include <pcl/io/png_io.h>                              //保存深度图像
using namespace pcl::console;
int main(int argc, char **argv)
{

    // Generate the data
    if (argc < 2)
    {

        print_error("Syntax is: %s input.pcd -w 640 -h 480 -cx 320 -cy 240 -fx 525 -fy 525 -type 0 -size 2\n", argv[0]);
        print_info("  where options are:\n");
        print_info("                     -w X = width of detph iamge ");

        return -1;
    }
    std::string filename = argv[1]; //将命令行的第二个参数传给filename

    //默认的参数
    int width = 640, height = 480, size = 2, type = 0;
    float fx = 525, fy = 525, cx = 320, cy = 240;

    //命令行解析
    //可以在命令行输入附加参数
    parse_argument(argc, argv, "-w", width);   //深度图像宽度
    parse_argument(argc, argv, "-h", height);  //深度图像高度
    parse_argument(argc, argv, "-cx", cx);     //光轴在深度图像上的x坐标
    parse_argument(argc, argv, "-cy", cy);     //光轴在深度图像上的y坐标
    parse_argument(argc, argv, "-fx", fx);     //水平方向焦距
    parse_argument(argc, argv, "-fy", fy);     //垂直方向焦距
    parse_argument(argc, argv, "-type", type); //曲面重建时三角化的方式
    parse_argument(argc, argv, "-size", size); //曲面重建时的面片的大小
    //convert unorignized point cloud to orginized point cloud begin
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile(filename, *cloud); //将命令行的第二个参数传递给filename之后，将filename对应的pcd文件读入
    pcl::io::loadPCDFile(filename, *cloud_in);
    print_info("Read pcd file successfully\n"); //读入成功标志

    //设置sensor_pose和coordinate_frame
    Eigen::Affine3f sensorPose; //设置相机位姿
    sensorPose.setIdentity();   //成像时遵循的坐标系统
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel = 0.00; //设置噪声水平
    float minRange = 0.0f;   //成像时考虑该阈值外的点

    pcl::RangeImagePlanar::Ptr rangeImage(new pcl::RangeImagePlanar);
    rangeImage->createFromPointCloudWithFixedSize(*cloud, width, height, cx, cy, fx, fy, sensorPose, coordinate_frame);
    std::cout << rangeImage << "\n";
    //convert unorignized point cloud to orginized point cloud end

    //保存深度图像
    float *ranges = rangeImage->getRangesArray();
    unsigned char *rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, rangeImage->width, rangeImage->height);
    //pcl::io::saveRgbPNGFile("rangeImage.png", rgb_image, rangeImage->width, rangeImage->height);
    //std::cerr << "rangeImage.png Saved!" << std::endl;

    //viusalization of range image、
    //深度图像可视化
    pcl::visualization::RangeImageVisualizer range_image_widget("rangeImage");
    range_image_widget.showRangeImage(*rangeImage);
    range_image_widget.setWindowTitle("PCL_rangeImage");
    //triangulation based on range image
    pcl::OrganizedFastMesh<pcl::PointWithRange>::Ptr tri(new pcl::OrganizedFastMesh<pcl::PointWithRange>);
    pcl::search::KdTree<pcl::PointWithRange>::Ptr tree(new pcl::search::KdTree<pcl::PointWithRange>);
    tree->setInputCloud(rangeImage);                                                                 //用到了C++赋值兼容原则，派生类的指针可以赋值给基类的指针
    pcl::PolygonMesh triangles;                                                                      //多边形Mesh
    tri->setTrianglePixelSize(size);                                                                 //曲面重建的精细程度
    tri->setInputCloud(rangeImage);                                                                  //设置输入的深度图像
    tri->setSearchMethod(tree);                                                                      //设置搜索方式
    tri->setTriangulationType((pcl::OrganizedFastMesh<pcl::PointWithRange>::TriangulationType)type); //设置三角化的类型，是一个枚举类型，将命令行输入的type值进行强制类型转换至对应的三角化的类型
    tri->reconstruct(triangles);                                                                     //重建结果传送至triangles

    //可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL"));
    //可视化重建结果  ViewPort v1
    int v1(0);
    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->setBackgroundColor(0.5, 0.5, 0.5, v1);
    viewer->addPolygonMesh(triangles, "tin", v1);

    //可视化原始点云
    int v2(0);
    viewer->createViewPort(0.5, 0, 1, 1, v2);
    viewer->setBackgroundColor(0, 128, 0, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud_in, 190, 0, 0);
    viewer->addPointCloud(cloud_in, color1, "cloud_in", v2);
    viewer->addCoordinateSystem();

    while (!range_image_widget.wasStopped() && !viewer->wasStopped())
    {
        range_image_widget.spinOnce();

        Sleep(0.01);
        viewer->spinOnce();
    }
}
