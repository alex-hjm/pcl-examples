#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/time.h>

int main(int argc, char** argv)
{
	if (argc != 2)
		return (0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr handledCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("room.pcd", *cloud);
	pcl::copyPointCloud(*cloud, *handledCloud);
	pcl::console::TicToc time; time.tic();

	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	std::vector <float> moment_of_inertia; //存储惯性矩的特征向量
	std::vector <float> eccentricity; //存储偏心率的特征向量
	//AABB
	pcl::PointXYZ min_point_AABB; //轴对齐包围盒
	pcl::PointXYZ max_point_AABB;
	//OBB
	pcl::PointXYZ min_point_OBB; //有向包围盒
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia(moment_of_inertia);//惯性矩特征
	feature_extractor.getEccentricity(eccentricity);//偏心率特征
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);//AABB对应的左下角和右下角坐标
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);//OBB对应的相关参数
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);//三个特征值
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);//三个特征向量
	feature_extractor.getMassCenter(mass_center);//点云中心坐标


	  // 可视化
	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
	viewer.initCameraParameters();
	pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
	
	//AABB
	int v1(0);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1);//背景色为冷灰色
	viewer.addText("AABB", 10, 10, "v1 test", v1);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);
	viewer.addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 0.0, 0.0, "AABB", v1);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB", v1);
	viewer.addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector", v1);
	viewer.addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector", v1);
	viewer.addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector", v1);

	//OBB
	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	std::cout << "position_OBB: " << position_OBB << endl;
	std::cout << "mass_center: " << mass_center << endl;
	Eigen::Quaternionf quat(rotational_matrix_OBB);
	int v2(0);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
	viewer.addText("OBB", 10, 10, "v2 test", v2);
	viewer.addPointCloud<pcl::PointXYZ>(handledCloud, "handledCloud", v2);
	viewer.addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB", v2);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB", v2);
	std::cout << "size of cloud :" << cloud->points.size() << endl;
	std::cout << "moment_of_inertia :" << moment_of_inertia.size() << endl;
	std::cout << "eccentricity :" << eccentricity.size() << endl;
	cout << "计算运行时间: " << time.toc() / 1000 << "s" << endl;



	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("点云库PCL学习教程第二版-基于惯性矩与偏心率的描述子"));
	viewer->setBackgroundColor(1, 1, 1);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->addPointCloud<pcl::PointXYZ>(cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 0, 255, 0), "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
	viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 0.0, 0.0, "AABB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "AABB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "AABB");
	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	std::cout << "position_OBB: " << position_OBB << endl;
	std::cout << "mass_center: " << mass_center << endl;
	Eigen::Quaternionf quat(rotational_matrix_OBB);
	viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "OBB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "OBB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "OBB");
	viewer->setRepresentationToWireframeForAllActors();
	pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
	viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
	std::cout << "size of cloud :" << cloud->points.size() << endl;
	std::cout << "moment_of_inertia :" << moment_of_inertia.size() << endl;
	std::cout << "eccentricity :" << eccentricity.size() << endl;*/
	//Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
	//Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
	//Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
	//Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
	//Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
	//Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
	//Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
	//Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

	//p1 = rotational_matrix_OBB * p1 + position;
	//p2 = rotational_matrix_OBB * p2 + position;
	//p3 = rotational_matrix_OBB * p3 + position;
	//p4 = rotational_matrix_OBB * p4 + position;
	//p5 = rotational_matrix_OBB * p5 + position;
	//p6 = rotational_matrix_OBB * p6 + position;
	//p7 = rotational_matrix_OBB * p7 + position;
	//p8 = rotational_matrix_OBB * p8 + position;

	//pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
	//pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
	//pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
	//pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
	//pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
	//pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
	//pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
	//pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

	//viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
	//viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
	//viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
	//viewer->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4 edge");
	//viewer->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5 edge");
	//viewer->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6 edge");
	//viewer->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7 edge");
	//viewer->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8 edge");
	//viewer->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9 edge");
	//viewer->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10 edge");
	//viewer->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11 edge");
	//viewer->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12 edge");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);
}
