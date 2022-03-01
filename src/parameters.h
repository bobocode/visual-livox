#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv/cv.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <cv_bridge/cv_bridge.h>


extern int IMG_HEIGHT;
extern int IMG_WIDTH;

extern Eigen::Matrix4f cam_2_laser;
extern Eigen::MatrixXf P_rect;

extern bool CLAHE;
extern bool optical_flow_match;
extern bool visualize_depth;
extern bool visualize_matched;


struct ImagePoint {
     float u, v;
     int ind;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (ImagePoint,
                                   (float, u, u)
                                   (float, v, v)
                                   (int, ind, ind))

struct DepthPoint {
     float u, v;
     float depth;
     int label;
     int ind;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (DepthPoint,
                                   (float, u, u)
                                   (float, v, v)
                                   (float, depth, depth)
                                   (int, label, label)
                                   (int, ind, ind))



#endif