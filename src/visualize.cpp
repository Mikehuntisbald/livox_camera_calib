#include "ceres/ceres.h"
#include "common.h"
#include "lidar_camera_calib.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "lidarCamCalib");

    ros::NodeHandle nh;
    vector<double> camera_matrix;
    vector<double> dist_coeffs;
    vector<double> extrinsic;
    std::string pcd_dir, jpg_dir;
    nh.param<vector<double>>("camera/camera_matrix", camera_matrix,
                               vector<double>());
    nh.param<vector<double>>("camera/dist_coeffs", dist_coeffs, vector<double>());
    nh.param<vector<double>>("camera/extrinsic", extrinsic, vector<double>());
    nh.param<std::string>("camera/pcd_dir", pcd_dir, std::string);
    nh.param<std::string>("camera/jpg_dir", jpg_dir, std::string);
    // Load point cloud from PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/media/sam/Extreme\ SSD/0.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read PCD file\n");
        return -1;
    }

    // Load image from JPG file
    cv::Mat image = cv::imread("/home/sam/.ros/0.jpg");
    if (image.empty()) {
        std::cerr << "Failed to load image." << std::endl;
        return -1;
    }
    
  // loadCameraConfig(image_file);
    cv::Mat camera_matrix_ =
      (cv::Mat_<double>(3, 3) << camera_matrix[0], 0.0, camera_matrix[2], 0.0, camera_matrix[4], camera_matrix[5], 0.0, 0.0, 1.0);
    cv::Mat dist_coeffs_ =
      (cv::Mat_<double>(1, 5) << dist_coeffs[0], dist_coeffs[1], dist_coeffs[2], dist_coeffs[3], dist_coeffs[4]);
    cv::Mat extrinsics =
      (cv::Mat_<double>(4, 4) << extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3], extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7], extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11], extrinsic[12], extrinsic[13], extrinsic[14], extrinsic[15]);
    // Camera intrinsics and extrinsics

    // cv::Mat undistortedImage;
    // // std::cout<<camera_matrix<<distortion_coeff<<std::endl;
    // cv::undistort(image, undistortedImage, camera_matrix_, dist_coeffs_);
    // undistortedImage.copyTo(image);

    //cv::Mat extrinsics = (cv::Mat_<double>(4, 4) <<
    //    0.0240456,-0.999707,-0.00270886,0.134656,
    //    0.0128452,0.00301838,-0.999913,-0.212597,
    //    0.999628,0.0240088,0.0129141,-0.0985589,
    //    0,0,0,1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& point : cloud->points) {
        cv::Mat point3D = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);
        cv::Mat pointCam = extrinsics * point3D;
        cv::Mat point2D = camera_matrix_ * pointCam.rowRange(0, 3);
        point2D /= point2D.at<double>(2, 0);

        int u = static_cast<int>(point2D.at<double>(0, 0));
        int v = static_cast<int>(point2D.at<double>(1, 0));

        if (u >= 0 && u < image.cols && v >= 0 && v < image.rows) {
            cv::Vec3b color = image.at<cv::Vec3b>(v, u);
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = color[2];
            colored_point.g = color[1];
            colored_point.b = color[0];
            colored_cloud->points.push_back(colored_point);
        }
    }

    colored_cloud->width = colored_cloud->points.size();
    colored_cloud->height = 1;
    colored_cloud->is_dense = false;

	pcl::visualization::PCLVisualizer viewer;  //显示点云
	viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud);
	viewer.addCoordinateSystem(1.0);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}

    return 0;
}
