#ifndef _POINT_CLOUD_FACTORY_H_
#define _POINT_CLOUD_FACTORY_H_

#include <ros/package.h>
#include <ros/ros.h>

#include <boost/lexical_cast.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "parameters.h"

class PointCloudFactory
{
public:
    PointCloudFactory()
    {
        print_result = false;
        downsample_grid_size = 5;
    }

    void projectPointCloud(const pcl::PointCloud<pcl::PointXYZ>& point_cloud_pcl);
    void downsamplePointCloud();
    cv::Mat visualizePointCloud(const cv::Mat& gray_image, const bool select_downsampled = true);
    float queryDepth(const float x, const float y, const int searching_radius = 2) const;
    cv::Mat visualizeDepth(const cv::Mat& gray_image);


    Eigen::MatrixXf point_cloud_3d_tilde;  // row size is dynamic, and will be decided when load the point cloud; column
                                         // size is fixed as 4
    Eigen::MatrixXf point_cloud_2d;
    Eigen::MatrixXf point_cloud_2d_dnsp;

    int downsample_grid_size;

    Eigen::MatrixXf bucket_x;
    Eigen::MatrixXf bucket_y;
    Eigen::MatrixXf bucket_depth;
    Eigen::MatrixXi bucket_count;

    cv::Mat image_with_point_cloud;
    cv::Mat image_with_depth;

    bool print_result;
    
     
};


#endif