#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include <boost/assign.hpp> //for operator +=
#include <boost/assert.hpp> //for BOOST_ASSERT
#include <boost/assign/std/vector.hpp> //for operator+ vector

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "livox_mapping/laserMapping.h"
#include "livox_mapping/scanRegistration.h"
#include "imageFactory.h"
#include "pointcloudFactory.h"
#include "ceresCostFunction.h"

#include "parameters.h"

int IMG_HEIGHT;
int IMG_WIDTH;
Eigen::Matrix4d cam_2_laser;
Eigen::MatrixXd P_rect;
bool CLAHE;
bool optical_flow_match;
bool visualize_depth;
bool visualize_matched;

ImageFactory img_factory;
PointCloudFactory cloud_factory;

cv::Mat last_img, cur_img, last_des, cur_des;
std::vector<cv::KeyPoint> last_keypoints, cur_keypoints,last_keypoints_2f, cur_keypoints_2f;
std::vector<uchar> optical_flow_status;
double last_time, cur_time;
std::vector<cv::DMatch> matches;

pcl::PointCloud<PointType>::Ptr laserCloud;
pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
pcl::PointCloud<PointType>::Ptr surfPointsFlat;

Eigen::Matrix4d cur_laser_2_init;
Eigen::Matrix4d last_laser_2_init;

ros::Publisher *voDataPubPointer = NULL;
ros::Publisher *imagePointsProjPubPointer = NULL;
ros::Publisher *depthPointsPubPointer = NULL;
ros::Publisher *imgageMatchedPubPointer = NULL;

std::mutex process_mtx;
std::shared_ptr<LaserMapping> cloud_mapping;
std::shared_ptr<ScanRegistration> cloud_registration;

tf2::Transform cam_cur_to_cur_last, cam_cur_to_cam_init, cam_last_to_cam_init;
tf2::Quaternion cam_cur_q_cur_last, cam_cur_q_cam_init;

double angles_last_to_cur[3];
double t_last_to_cur[3];

bool first_img;

void init()
{
  cloud_registration = std::make_shared<ScanRegistration>();
  cloud_mapping = std::make_shared<LaserMapping>();

  cloud_registration->init();
  cloud_mapping->init();

  laserCloud = boost::make_shared<pcl::PointCloud<PointType>>();
  cornerPointsSharp = boost::make_shared<pcl::PointCloud<PointType>>();
  surfPointsFlat = boost::make_shared<pcl::PointCloud<PointType>>();

  first_img = true;

  cur_laser_2_init.setIdentity();
  last_laser_2_init.setIdentity();

  cam_cur_to_cam_init.setIdentity();
  cam_last_to_cam_init.setIdentity();

  for(int i =0; i < 3; i++)
  {
    angles_last_to_cur[i] = 0.0;
    t_last_to_cur[i] = 0.0;
  }

}

void processImage(const sensor_msgs::Image::ConstPtr& image_msg)
{
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

  if(CLAHE)
  {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0);
    clahe->apply(cv_ptr->image, cur_img);
  }else
  {
    cur_img = cv_ptr->image;

  }

  cur_keypoints.clear();
  cur_keypoints = img_factory.detKeypoints(cur_img);

  if(!optical_flow_match)
  {
    cur_des = img_factory.descKeypoints(cur_keypoints, cur_img);
  }

  if(!first_img)
  {
    if(!optical_flow_match)
    {
      matches = img_factory.matchDescriptors(last_des, cur_des);
    }else
    {

      optical_flow_status.clear();
      last_keypoints_2f.clear();
      cur_keypoints_2f.clear();
      std::tie(last_keypoints_2f, cur_keypoints_2f, optical_flow_status) = img_factory.calculateOpticalFlow(last_img, cur_img, cur_keypoints);

    }


    if(visualize_matched)
    {
      cv::Mat matched_img;
      std_msgs::Header header;
      header.stamp = ros::Time().fromSec(cur_time);

      if(!optical_flow_match)
      {
        matched_img = img_factory.visualizeMatches(last_img, cur_img, last_keypoints, cur_keypoints,matches);

      }else
      {
        matched_img = img_factory.visualizeOpticalFlow(cur_img, last_keypoints_2f, cur_keypoints_2f, optical_flow_status);

      }

      cv_bridge::CvImage matched_viz_cvbridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, matched_img);
      imgageMatchedPubPointer->publish(matched_viz_cvbridge.toImageMsg());
    }
    
  }

  last_img = cur_img;
  last_des = cur_des;
  last_keypoints_2f = cur_keypoints_2f;
  last_keypoints = cur_keypoints;


}

void processImageProj(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ> point_cloud_pcl;
  pcl::fromROSMsg(*point_cloud_msg, point_cloud_pcl);

  cloud_factory.projectPointCloud(point_cloud_pcl);

  if(visualize_depth)
  {
    cv::Mat img_with_depth = cloud_factory.visualizeDepth(cur_img);
    std_msg::Header header;
    header.stamp = ros::Time().fromSec(cur_time);
    cv_bridge::CvImage depth_viz_cvbridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img_with_depth);
    imagePointsProjPubPointer->publish(depth_viz_cvbridge.toImageMsg());

  }

}

void laserIO(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg)
{
  cloud_registration->laserCloudHandler(point_cloud_msg);
  cloud_registration->cloudOut(laserCloud, cornerPointsSharp, surfPointsFlat);

  cloud_mapping->cloudIn(laserCloud, cornerPointsSharp, surfPointsFlat);
  cloud_mapping->resultPublish();

  cur_laser_2_init = cloud_mapping->laser_to_init;

  Eigen::Matrix4d cur_laser_to_last_laser = cur_laser_2_init.inverse() * last_laser_2_init;
  
  Eigen::Matrix4d cur_cam_2_last_cam = cam_2_laser * cur_laser_to_last_laser * cam_2_laser.inverse();

  Eigen::Affine3d eigen_transform;
  eigen_transform.matrix() = cur_cam_2_last_cam;

  geometry_msgs::TransformStamped tmp = tf2::tf2::eigenToTransform(eigen_transform);

  tf2::fromMsg(tmp, cam_cur_to_cur_last);
}

void solveOdometry()
{
  ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
  ceres::Problem problem;

  if(!first_img)
  {
    t_last_to_cur[0] = cam_cur_to_cur_last.getOrigin().getX();
    t_last_to_cur[1] = cam_cur_to_cur_last.getOrigin().getY();
    t_last_to_cur[2] = cam_cur_to_cur_last.getOrigin().getZ();

    angles_last_to_cur[0] = cam_cur_to_cur_last.getRotation().getAxis().getX() *
                     cam_cur_to_cur_last.getRotation().getAngle();
    angles_last_to_cur[1] = cam_cur_to_cur_last.getRotation().getAxis().getY() *
                     cam_cur_to_cur_last.getRotation().getAngle();
    angles_last_to_cur[2] = cam_cur_to_cur_last.getRotation().getAxis().getZ() *
                     cam_cur_to_cur_last.getRotation().getAngle();

    int prev_pt_x, prev_pt_y, cur_pt_x, cur_pt_y;
    int counter33 = 0, counter32 =0, counter23 = 0, counter22 = 0;
    int match_num = (optical_flow_match) ? cur_keypoints_2f.size(): matches.size();

    for(int i = 0; i < match_num; i++)
    {
      if(!optical_flow_match)
      {
        prev_pt_x = last_keypoints[matches[i].queryIdx].pt.x;
        prev_pt_y = last_keypoints[matches[i].queryIdx].pt.y;
        cur_pt_x = cur_keypoints[matches[i].trainIdx].pt.x;
        cur_pt_y = cur_keypoints[matches[i].trainIdx].pt.y;
      }else
      {
        if(optical_flow_status[i] == 1)
        {
          prev_pt_x = last_keypoints_2f[i].x;
          prev_pt_y = last_keypoints_2f[i].y;

          cur_pt_x = cur_keypoints_2f[i].x;
          cur_pt_y = cur_keypoints_2f[i].y;
        }else
        {
          continue;
        }
      }

      float last_depth = cloud_factory.queryDepth(prev_pt_x, prev_pt_y);
      float cur_depth = cloud_factory.queryDepth(cur_pt_x, cur_pt_y);

      Eigen::Vector3f last_img_point3d;
      Eigen::Vector3f cur_img_point3d;

      Eigen::Vector3f last_rect3d;
      Eigen::Vector3f cur_rect3d;

      if(last_depth > 0)
      {
        last_img_point3d << prev_pt_x * last_depth, prev_pt_y * last_depth, last_depth;
        cur_img_point3d << cur_pt_x, cur_pt_y, 1.0f;

        last_rect3d = P_rect.leftCols(3).colPivHouseholderQr().solve(last_img_point3d);
        cur_rect3d = P_rect.leftCols(3).colPivHouseholderQr().solve(cur_img_point3d);

        ceres::CostFunction* cost_function = CostFunctor32::Create(
          static_cast<double>(last_rect3d(0)), static_cast<double>(last_rect3d(1)),static_cast<double>(last_rect3d(2)),
          static_cast<double>(cur_rect3d(0)) / static_cast<double>(cur_rect3d(2)),
          static_cast<double>(cur_rect3d(1)) / static_cast<double>(cur_rect3d(2)));
        
        problem.AddResidualBlock(cost_function, loss_function, angles_last_to_cur, t_last_to_cur);
        ++counter32;
      }else
      {
        last_img_point3d << prev_pt_x, prev_pt_y, 1.0f;
        cur_img_point3d << cur_pt_x, cur_pt_y, 1.0f;

        last_rect3d = P_rect.leftCols(3).colPivHouseholderQr().solve(last_img_point3d);
        cur_rect3d = P_rect.leftCols(3).colPivHouseholderQr().solve(cur_img_point3d);

        ceres::CostFunction* cost_function = CostFunctor22::Create(
          static_cast<double>(last_rect3d(0))/static_cast<double>(last_rect3d(2)), 
          static_cast<double>(last_rect3d(1))/static_cast<double>(last_rect3d(2)),
          static_cast<double>(cur_rect3d(0)) / static_cast<double>(cur_rect3d(2)),
          static_cast<double>(cur_rect3d(1)) / static_cast<double>(cur_rect3d(2)));
        
        problem.AddResidualBlock(cost_function, loss_function, angles_last_to_cur, t_last_to_cur);
        ++counter22

      }

    }
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cam_cur_to_cur_last.setOrigin(tf2::Vector3(t_last_to_cur[0], t_last_to_cur[1],t_last_to_cur[2]));
    double angle = std::sqrt(std::pow(angles_last_to_cur[0], 2) + std::pow(angles_last_to_cur[1], 2) + std::pow(angles_last_to_cur[2], 2));
    cam_cur_q_cur_last.setRotation(
        tf2::Vector3(angles_last_to_cur[0] / angle, angles_last_to_cur[1] / angle, angles_last_to_cur[2] / angle), angle);
    cam_cur_to_cur_last.setRotation(cam_cur_q_cur_last);

  }

  cam_cur_to_cam_init = cam_last_to_cam_init * cam_cur_to_cur_last;
  cam_last_to_cam_init = cam_cur_to_cam_init;
  
  Eigen::Quaterniond tmp_q(cam_cur_to_cam_init.getRotation());
  Eigen::Vector3d tmp_T(cam_cur_to_cam_init.getOrigin());

  nav_msgs::Odometry visualOdom;
  visualOdom.header.frame_id = "/camera_init";
  visualOdom.child_frame_id = "/visual_optimized";

  visualOdom.header.stamp = ros::Time().fromSec(cur_time);
  visualOdom.pose.pose.orientation.x = tmp_q.x;
  visualOdom.pose.pose.orientation.y = tmp_q.y;
  visualOdom.pose.pose.orientation.z = tmp_q.z;
  visualOdom.pose.pose.orientation.w = tmp_q.w;
  visualOdom.pose.pose.position.x = tmp_T.x;
  visualOdom.pose.pose.position.y = tmp_T.y;
  visualOdom.pose.pose.position.z = tmp_T.z;

  voDataPubPointer->publish(visualOdom);

}

void callback(const sensor_msgs::Image::ConstPtr& image_msg,
              const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg)
{

  process_mtx.lock();
  
  cur_time = image_msg->header.stamp.toSec();
  //Section 1: process Image
  processImage(image_msg);

  //Section 2: process depth cloud
  processImageProj(point_cloud_msg);

  //Section 3: solve odometry
  laserIO(point_cloud_msg);

  if(!first_img)
  {
    solveOdometry();
  }

  last_time = cur_time;

  if(first_img)
  {
    first_img = false;
  }

  process_mtx.unlock();

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualOdometry");
  ros::NodeHandle nh;

  std::string config_file;
  nh.getParam("feature_config_file", config_file);
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  std::cerr << "ERROR: Wrong path to settings" << std::endl;
  usleep(100);

  fs["img_width"] >> IMG_WIDTH;
  fs["img_height"] >> IMG_HEIGHT;
  fs["clahe"] >> CLAHE;
  fs["optical_flow"] >> optical_flow_match;

  cv::Mat cv_cam_T_laser;
  fs["cam_T_laser"] >> cv_cam_T_laser;
  cv::cv2eigen(cv_cam_T_laser, cam_2_laser);

  cv::Mat cv_cam_p;
  fs["cam_P"] >> cv_cam_p;
  cv::cv2eigen(cv_cam_p, P_rect);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::PointCloud2>  MySyncPolicy;

  message_filters::Subscriber<sensor_msgs::Image> sub_image(nh, "/image_raw", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_laser(nh, "/livox/lidar", 10);

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), sub_image,sub_laser);
  sync.setInterMessageLowerBound(ros::Duration(0.09));
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  ros::Publisher voDataPub = nh.advertise<nav_msgs::Odometry> ("/cam_to_init", 5);
  voDataPubPointer = &voDataPub;

  ros::Publisher imagePointsProjPub = nh.advertise<sensor_msgs::Image>("/image_proj_show", 1);
  imagePointsProjPubPointer = &imagePointsProjPub;

  ros::Publisher imgageMatchedPub = nh.advertise<sensor_msgs::Image>("/image_matched_show", 1);
  imgageMatchedPubPointer = &imgageMatchedPub;

  ros::spin();

  return 0;
}