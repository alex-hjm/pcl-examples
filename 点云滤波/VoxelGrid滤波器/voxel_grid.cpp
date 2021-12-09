//****使用VoxelGrid滤波器对点云进行下采样****//

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // 填入点云数据
    pcl::io::loadPCDFile("demo.pcd", *cloud);

    pcl::VoxelGrid<pcl::PointXYZ> sor; // 创建滤波器对象
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.003, 0.003, 0.003); //设置三维栅格的大小，数值越大，栅格越大，采样后的点云数据越小
    sor.filter(*cloud_filtered);          //cloud_filtered——滤波后的点云

    // 可视化
    pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
    viewer.initCameraParameters();

    int v1(0);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1); //背景色为冷灰色
    viewer.addText("Cloud before voxelgrid filtering", 10, 10, "v1 test", v1);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);

    int v2(0);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
    viewer.addText("Cloud after voxelgrid filtering", 10, 10, "v2 test", v2);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, "cloud_filtered", v2);
    pcl::io::savePLYFileBinary("demo1.ply", *cloud_filtered);

    while (!viewer.wasStopped())
    {
        //使可视化窗口不会一闪而过
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return (0);
}
