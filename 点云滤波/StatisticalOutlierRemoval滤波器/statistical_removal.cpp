#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char **argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // 填入点云数据
    pcl::io::loadPCDFile("room_scan1.pcd", *cloud);
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // 创建滤波器对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);            //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh(1.0); //设置判断是否为离群点的阈值
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    // 可视化
    pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
    viewer.initCameraParameters();

    int v1(0);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1); //背景色为冷灰色
    viewer.addText("Cloud before StatisticalOutlierRemoval filtering", 10, 10, "v1 test", v1);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);

    int v2(0);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
    viewer.addText("Cloud after StatisticalOutlierRemoval filtering", 10, 10, "v2 test", v2);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, "cloud_filtered", v2);

    while (!viewer.wasStopped())
    {
        //使可视化窗口不会一闪而过
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return (0);
}
