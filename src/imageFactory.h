#ifndef _IMAGE_FACTORY_H_
#define _IMAGE_FACTORY_H_

#include <ros/package.h>
#include <ros/ros.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "parameters.h"

enum class DetectorType
{
  ShiTomasi,
  BRISK,
  FAST,
  ORB,
  AKAZE,
  SIFT
};
static const std::string DetectorType_str[] = { "ShiTomasi", "BRISK", "FAST", "ORB", "AKAZE", "SIFT" };
enum class DescriptorType
{
  BRISK,
  ORB,
  BRIEF,
  AKAZE,
  FREAK,
  SIFT
};
static const std::string DescriptorType_str[] = { "BRISK", "ORB", "BRIEF", "AKAZE", "FREAK", "SIFT" };
enum class MatcherType
{
  BF,
  FLANN
};
enum class SelectType
{
  NN,
  KNN
};

class ImageFactory
{
public:
    
    ImageFactory()
    {
        print_result = false;
        visualize_result = false;
        detector_type = DetectorType::ShiTomasi;
        descriptor_type = DescriptorType::ORB;
        matcher_type = MatcherType::BF;
        selector_type = SelectType::NN;
    }

    std::vector<cv::KeyPoint> detKeypoints(cv::Mat &img);
    std::vector<cv::KeyPoint> keyPointsNMS(std::vector<cv::KeyPoint> &&keypoints,
                                         const int bucket_width = 100,  // width for horizontal direction in image plane
                                                                        // => x, col
                                         const int bucket_height = 100,  // height for vertical direction in image plane
                                                                         // => y, row
                                         const int max_total_keypoints = 400);
    cv::Mat descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img);
    std::vector<cv::DMatch> matchDescriptors(cv::Mat &desc_source, cv::Mat &desc_ref);
    void visualizeMatchesCallBack(int event, int x, int y);
    cv::Mat visualizeMatches(const cv::Mat &image0, const cv::Mat &image1, const std::vector<cv::KeyPoint> &keypoints0,
                           const std::vector<cv::KeyPoint> &keypoints1, const std::vector<cv::DMatch> &matches);
    std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>, std::vector<uchar>>
    calculateOpticalFlow(const cv::Mat &image0, const cv::Mat &image1, const std::vector<cv::KeyPoint> &keypoints0);
    cv::Mat visualizeOpticalFlow(const cv::Mat &image1, const std::vector<cv::Point2f> &keypoints0_2f,
                               const std::vector<cv::Point2f> &keypoints1_2f,
                               const std::vector<uchar> &optical_flow_status);
    cv::Mat visualizeOpticalFlow(const cv::Mat &image1, const std::vector<cv::KeyPoint> &keypoints0,
                               const std::vector<cv::KeyPoint> &keypoints1, const std::vector<cv::DMatch> &matches);
    
    bool print_result;
    bool visualize_result;
    DetectorType detector_type;
    DescriptorType descriptor_type;
    MatcherType matcher_type;
    SelectType selector_type;

    int remove_VO_outlier;
    bool optical_flow_match;

private:
    double time;
    cv::Mat img_keypoints;

};


#endif