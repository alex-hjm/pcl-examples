#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/cloud_viewer.h>
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("room_scan1.pcd", *cloud);

    // 创建一个系数为X=Y=0,Z=1的平面
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;
    // 创建滤波器对象
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_filtered);

    // 可视化
    pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
    viewer.initCameraParameters();

    int v1(0);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1); //背景色为冷灰色
    viewer.addText("Cloud before passthrough filtering", 10, 10, "v1 test", v1);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);

    int v2(0);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
    viewer.addText("Cloud after passthrough filtering", 10, 10, "v2 test", v2);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, "cloud_filtered", v2);

    viewer.addCoordinateSystem();

    while (!viewer.wasStopped())
    {
        //使可视化窗口不会一闪而过
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
