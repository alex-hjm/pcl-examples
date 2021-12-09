#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
int main()
{
    //加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("table_scene_mug_stereo_textured.pcd", *cloud);
    //估计法线
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT); //设置估计方法
    //法线估计方法
    //COVARIANCE_MATRIX模式从具体某个点的局部邻域的协方差矩阵创建9个积分图，来计算这个点的法线
    //AVERAGE_3D_GRADIENT模式创建了6个积分图来计算水平和垂直方向的平滑后的三维梯度，并使用两个梯度间的向量积计算法线
    //AVERAGE_DEPTH_CHANGE模式只创建了一个单一的积分图，并从平均深度变化计算法线
    ne.setMaxDepthChangeFactor(0.02f); //最大深度变化系数
    ne.setNormalSmoothingSize(10.0f);  //优化法线方向时考虑邻域大小
    ne.setInputCloud(cloud);           //输入点云，必须为有序点云
    ne.compute(*normals);              //执行法线估计存储结果到normals
    //法线可视化
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.initCameraParameters();
    int v1(0); //显示原始数据
    int v2(0); //显示法线估计
    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPort(0.5, 0, 1, 1, v2);
    viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1); //原始点云
    viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 100, 0.02f, "cloud_normals", v2); //将法线显示在第二个窗口
    viewer.initCameraParameters();
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud_in", v1); //第一个窗口显示原始点云
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 156, 0, "cloud_in", v1);//原始点云颜色
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}
