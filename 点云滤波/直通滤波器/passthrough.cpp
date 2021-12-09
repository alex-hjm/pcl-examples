#include <iostream>
#include <pcl/io/pcd_io.h>
//#include <ctime>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("room_scan1.pcd", *cloud);

    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass; //设置滤波器对象
    pass.setInputCloud(cloud);            //设置输入点云
    pass.setFilterFieldName("z");         //设置过滤时所需点云类型的z字段
    pass.setFilterLimits(0.0, 10);        //设置过滤字段上的范围
    //pass.setFilterLimitsNegative (true); //设置保留范围内的还是过滤范围内的 true：过滤，false：保留
    pass.filter(*cloud_filtered); //执行过滤，过滤结果保存在cloud_filtered

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
