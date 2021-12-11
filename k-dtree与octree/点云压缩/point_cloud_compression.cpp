/*
功能：利用octree进行点云压缩和解压
*/

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include <iostream>

int main()
{
	// 加载点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	if (pcl::io::loadPCDFile("room_scan1.pcd", *sourceCloud) == -1)
		return -1;

	// 是否查看压缩信息
	bool showStatistics = true;
	// 配置文件，如果想看配置文件的详细内容，可以参考: /io/include/pcl/compression/compression_profiles.h
	pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

	// 初始化点云压缩器和解压器
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
	PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compressionProfile, showStatistics);
	PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compressionProfile, true, 0.002);


	// 压缩结果stringstream
	std::stringstream compressedData;
	// 输出点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());
	// 压缩点云
	PointCloudEncoder->encodePointCloud(sourceCloud, compressedData);
	std::cout << compressedData.str() << std::endl;
	// 解压点云
	PointCloudEncoder->decodePointCloud(compressedData, cloudOut);
   // 可视化
	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
	viewer.initCameraParameters();

	int v1(0);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1);//背景色为冷灰色
	viewer.addText("Cloud before voxelgrid filtering", 10, 10, "v1 test", v1);
	viewer.addPointCloud<pcl::PointXYZRGB>(sourceCloud, "cloud", v1);

	int v2(0);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
	viewer.addText("Cloud after voxelgrid filtering", 10, 10, "v2 test", v2);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloudOut, "cloud_filtered", v2);

	while (!viewer.wasStopped())
	{
		//使可视化窗口不会一闪而过
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/openni_grabber.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/compression/octree_pointcloud_compression.h>
//#include <stdio.h>
//#include <sstream>
//#include <stdlib.h>
//
//#ifdef WIN32
//# define sleep(x) Sleep((x)*1000)
//#endif
//
//class SimpleOpenNIViewer
//{
//public:
//SimpleOpenNIViewer () :
//viewer (" Point Cloud Compression Example")
//  {
//  }
//
//void
//cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
//  {
//if (!viewer.wasStopped ())
//    {
//// 存储压缩点云的字节流
//std::stringstream compressedData;
//// 输出点云
//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGBA> ());
//// 压缩点云
//PointCloudEncoder->encodePointCloud (cloud, compressedData);
//// 解压缩点云
//PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
////可视化解压缩点云
//viewer.showCloud (cloudOut);
//    }
//  }
//
//void
//run ()
//  {
//bool showStatistics=true;
//// 压缩选项详见 /io/include/pcl/compression/compression_profiles.h
//pcl::octree::compression_Profiles_e compressionProfile=pcl::octree::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
//// 初始化压缩与解压缩对象
//PointCloudEncoder=new pcl::octree::PointCloudCompression<pcl::PointXYZRGBA> (compressionProfile, showStatistics);
//PointCloudDecoder=new pcl::octree::PointCloudCompression<pcl::PointXYZRGBA> ();
////创建从 OpenNI获取点云的对象
//pcl::Grabber* interface =new pcl::OpenNIGrabber ();
////建立回调函数
//    boost::function<void
//   (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
//// 建立回调函数与回调信号之间联系
//boost::signals2::connection c = interface->registerCallback (f);
//// 开始接收点云数据流
//interface->start ();
//while (!viewer.wasStopped ())
//    {
//sleep (1);
//    }
//interface->stop ();
//// 删除点云压缩与解压缩对象实例
//delete (PointCloudEncoder);
//delete (PointCloudDecoder);
//  }
//pcl::visualization::CloudViewer viewer;
//pcl::octree::PointCloudCompression<pcl::PointXYZRGBA>*PointCloudEncoder;
//pcl::octree::PointCloudCompression<pcl::PointXYZRGBA>*PointCloudDecoder;
//};
//
//int
//main (int argc, char**argv)
//{
//SimpleOpenNIViewer v;
//v.run ();
//return (0);
//}
