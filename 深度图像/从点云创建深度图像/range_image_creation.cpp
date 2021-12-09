#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

void setViewerPose(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose)
{
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
    viewer.setCameraPosition(
        pos_vector[0],
        pos_vector[1],
        pos_vector[2],
        look_at_vector[0],
        look_at_vector[1],
        look_at_vector[2],
        up_vector[0],
        up_vector[1],
        up_vector[2]);
}

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ> pointCloud;
    //生成数据
    for (float y = -0.5f; y <= 0.5f; y += 0.01f)
    {
        for (float z = -0.5f; z <= 0.5f; z += 0.01f)
        {
            pcl::PointXYZ point;
            point.x = 2.0f - y;
            point.y = y;
            point.z = z;
            pointCloud.points.push_back(point);
        }
    }
    pointCloud.width = (uint32_t)pointCloud.points.size();
    pointCloud.height = 1;
    /*以1度为角分辨率，从上面创建的点云创建深度图像。*/
    float angularResolution = (float)(1.0f * (M_PI / 180.0f)); // 1度转弧度
    /*模拟深度传感器在水平方向的最大采样角度:360°视角。*/
    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f)); // 360.0度转弧度
    /*模拟深度传感器在垂直方向的最大采样角度:180°视角。*/
    float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f)); // 180.0度转弧度
    /*
    sensorPose定义模拟深度传感器的6自由度位置：
    横滚角roll、俯仰角pitch、偏航角yaw，默认初始值均为0。
    */
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    /*
    定义坐标轴正方向：
    CAMERA_FRAME：X轴向右，Y轴向下，Z轴向前；
    LASER_FRAME：X轴向前，Y轴向左，Z轴向上；
    */
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    /*
    noiseLevel设置周围点对当前点深度值的影响：
    如 noiseLevel=0.05，可以理解为，深度距离值是通过查询点半径为 Scm 的圆内包含的点用来平均计算而得到的。
    */
    float noiseLevel = 0.00;
    /*
    min_range设置最小的获取距离，小于最小获取距离的位置为盲区
    */
    float minRange = 0.0f;
    /*border_size获得深度图像的边缘的宽度：在裁剪图像时，如果 borderSize>O ，将在图像周围留下当前视点不可见点的边界*/
    int borderSize = 1;

    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage &range_image = *range_image_ptr;
    range_image.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    std::cout << range_image << "\n";

    //创建3D视图并且添加点云进行显示
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
    viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");

    viewer.initCameraParameters();
    setViewerPose(viewer, range_image.getTransformationToWorldSystem());

    //显示深度图像
    pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
    //range_image_widget.setRangeImage(range_image);
    range_image_widget.showRangeImage(range_image); //图像可视化方式显示深度图像
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
    //主循环
    while (!viewer.wasStopped())
    {
        range_image_widget.spinOnce();
        viewer.spinOnce();
        pcl_sleep(0.01);
        if (1)
        {
            scene_sensor_pose = viewer.getViewerPose();
            range_image.createFromPointCloud(pointCloud,
                                             angularResolution, pcl::deg2rad(360.0f),
                                             pcl::deg2rad(180.0f),
                                             scene_sensor_pose,
                                             pcl::RangeImage::LASER_FRAME,
                                             noiseLevel,
                                             minRange,
                                             borderSize);
            range_image_widget.showRangeImage(range_image);
        }
    }
}
