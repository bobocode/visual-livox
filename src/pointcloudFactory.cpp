#include "pointcloudFactory.h"


void PointCloudFactory:: projectPointCloud(const pcl::PointCloud<pcl::PointXYZ>& point_cloud_pcl)
{
    
  point_cloud_3d_tilde = Eigen::MatrixXf::Ones(point_cloud_pcl.size(), 4);
  for (j = 0; j < point_cloud_pcl.size(); ++j)
  {
    point_cloud_3d_tilde(j, 0) = point_cloud_pcl.points[j].x;
    point_cloud_3d_tilde(j, 1) = point_cloud_pcl.points[j].y;
    point_cloud_3d_tilde(j, 2) = point_cloud_pcl.points[j].z;
  }
  
  Eigen::MatrixXf point_cloud_3d = point_cloud_3d_tilde * cam_2_laser.transpose() * P_rect.transpose();
  Eigen::VectorXi select_front = (point_cloud_3d.col(2).array() > 0.1).cast<int>();  
  Eigen::MatrixXf point_cloud_3d_front(select_front.sum(), 3);

  int i, j = 0;
  for (i = 0; i < point_cloud_3d.rows(); ++i)
  {
      if (select_front(i))
      {
          point_cloud_3d_front.row(j) = point_cloud_3d.row(i);
          ++j;
      } 
  }

    // From homogeneous to normal coordiante, last column is still depth
  point_cloud_2d = point_cloud_3d_front;
  point_cloud_2d.leftCols(2) = point_cloud_2d.leftCols(2).array().colwise() * Eigen::inverse(point_cloud_2d.col(2).array());

}

void PointCloudFactory::downsamplePointCloud()
{
  const int new_width = std::ceil(static_cast<float>(IMG_WIDTH) / static_cast<float>(downsample_grid_size));
  const int new_height = std::ceil(static_cast<float>(IMG_HEIGHT) / static_cast<float>(downsample_grid_size));
  bucket_x = Eigen::MatrixXf::Zero(new_width, new_height);  // TODO: check if these lines are costly
  bucket_y = Eigen::MatrixXf::Zero(new_width, new_height);
  bucket_depth = Eigen::MatrixXf::Zero(new_width, new_height);
  bucket_count = Eigen::MatrixXi::Zero(new_width, new_height);

  int index_x, index_y, i, j, global_count = 0;
  for (i = 0; i < point_cloud_2d.rows(); ++i)
  {
    index_x = static_cast<int>(point_cloud_2d(i, 0) / downsample_grid_size);
    index_y = static_cast<int>(point_cloud_2d(i, 1) / downsample_grid_size);
    if (index_x >= 0 and index_x < new_width and index_y >= 0 and index_y < new_height)
    {
      if (bucket_count(index_x, index_y) == 0)
      {
        bucket_x(index_x, index_y) = point_cloud_2d(i, 0);
        bucket_y(index_x, index_y) = point_cloud_2d(i, 1);
        bucket_depth(index_x, index_y) = point_cloud_2d(i, 2);
        ++global_count;
      }
      else
      {  // incremental averaging -> TODO: check better averaging method
        bucket_x(index_x, index_y) +=
            (point_cloud_2d(i, 0) - bucket_x(index_x, index_y)) / bucket_count(index_x, index_y);
        bucket_y(index_x, index_y) +=
            (point_cloud_2d(i, 1) - bucket_y(index_x, index_y)) / bucket_count(index_x, index_y);
        bucket_depth(index_x, index_y) +=
            (point_cloud_2d(i, 2) - bucket_depth(index_x, index_y)) / bucket_count(index_x, index_y);
      }
      ++bucket_count(index_x, index_y);
    }
  }

  if (print_result)
    std::cout << "Point number in FOV after downsample = " << global_count << "\n" << std::endl;
  // grid size = 5 => print \approx 9.5k => ~10 times less points

  point_cloud_2d_dnsp = Eigen::MatrixXf(global_count, 3);  // TODO: check if this affect performance
  for (i = 0; i < new_width; ++i)
  {
    for (j = 0; j < new_height; ++j)
    {
      if (bucket_count(i, j) > 0)
      {
        --global_count;
        point_cloud_2d_dnsp(global_count, 0) = bucket_x(i, j);
        point_cloud_2d_dnsp(global_count, 1) = bucket_y(i, j);
        point_cloud_2d_dnsp(global_count, 2) = bucket_depth(i, j);
      }
    }
  }
  assert(global_count == 0);

}

cv::Mat PointCloudFactory::visualizePointCloud(const cv::Mat& gray_image, const bool select_downsampled = true)
{
  image_with_point_cloud = gray_image.clone();

  assert(image_with_point_cloud.size().height == IMG_HEIGHT);
  assert(image_with_point_cloud.size().width == IMG_WIDTH);

  cv::cvtColor(image_with_point_cloud, image_with_point_cloud, cv::COLOR_GRAY2BGR);

  Eigen::MatrixXf& point_cloud_2d_ = (select_downsampled) ? point_cloud_2d_dnsp : point_cloud_2d;

  float depth, depth_min = 0.1f, depth_max = 50.0f, ratio;
  int i = 0;
  for (i = 0; i < point_cloud_2d_.rows(); ++i)
  {
    depth = point_cloud_2d_(i, 2);
    ratio = std::max(std::min((depth - depth_min) / (depth_max - depth_min), 1.0f), 0.0f);
    if (ratio < 0.5)
    {
      cv::circle(image_with_point_cloud, cv::Point(point_cloud_2d_(i, 0), point_cloud_2d_(i, 1)),  // x, y
                 1, cv::Scalar(0, 255 * ratio * 2, 255 * (1 - ratio * 2)), cv::FILLED, cv::LINE_8);
    }
    else
    {
      cv::circle(image_with_point_cloud, cv::Point(point_cloud_2d_(i, 0), point_cloud_2d_(i, 1)), 1,
                 cv::Scalar(255 * (ratio - 0.5) * 2, 255 * (1 - (ratio - 0.5) * 2), 0), cv::FILLED, cv::LINE_8);
    }
  }

  return image_with_point_cloud;

}

float PointCloudFactory::queryDepth(const float x, const float y, const int searching_radius = 2) const
{
  // grid size and searching radius are respectively recommended to be 5 and 2
  assert(std::ceil(static_cast<float>(IMG_WIDTH) / static_cast<float>(downsample_grid_size)) == bucket_x.rows());
  assert(std::ceil(static_cast<float>(IMG_HEIGHT) / static_cast<float>(downsample_grid_size)) == bucket_x.cols());

  // float x = static_cast<float>(c);
  // float y = static_cast<float>(IMG_HEIGHT - r);
  int index_x = static_cast<int>(x / downsample_grid_size);
  int index_y = static_cast<int>(y / downsample_grid_size);
  const int new_width = bucket_x.rows();  // cautious, bucket axis0 is x, axis1 is y => different from image array
  const int new_height = bucket_x.cols();

  // select all neighbors in a certain local block
  int index_x_, index_y_;
  std::vector<Eigen::Vector4f> neighbors;
  Eigen::Vector4f neighbor;
  for (index_x_ = index_x - searching_radius; index_x_ <= index_x + searching_radius; ++index_x_)
  {
    for (index_y_ = index_y - searching_radius; index_y_ <= index_y + searching_radius; ++index_y_)
    {
      if (index_x_ >= 0 and index_x_ < new_width and index_y_ >= 0 and index_y_ < new_height and
          bucket_count(index_x_, index_y_) > 0)
      {
        neighbor(0) = bucket_x(index_x_, index_y_);
        neighbor(1) = bucket_y(index_x_, index_y_);
        neighbor(2) = bucket_depth(index_x_, index_y_);
        neighbor(3) = std::sqrt(std::pow(x - neighbor(0), 2) + std::pow(y - neighbor(1), 2));
        neighbors.push_back(neighbor);
        // std::cout << neighbor.transpose() << std::endl;
      }
    }
  }

  // edge case, no enough neighbors
  if (neighbors.size() < 10)
    return -1.0f;  // a fixed unrealistic value representing query failure

  // sort the vector; better ways can be quick select and heapify
  std::sort(neighbors.begin(), neighbors.end(),
            [&](const Eigen::Vector4f& n1, const Eigen::Vector4f& n2) -> bool { return n1(3) < n2(3); });

  // // Condition to be satisfied:  point x,y should be inside of the n0, n1 and n2 triangle
  // Eigen::Vector2f n0to1 = neighbors[1].head(2) - neighbors[0].head(2);
  // Eigen::Vector2f n0to2 = neighbors[2].head(2) - neighbors[0].head(2);
  // Eigen::Vector2f n0toP;
  // n0toP << x - neighbors[0](0), y - neighbors[0](1);
  // Eigen::Vector2f n1to0 = neighbors[0].head(2) - neighbors[1].head(2);
  // Eigen::Vector2f n1to2 = neighbors[2].head(2) - neighbors[1].head(2);
  // Eigen::Vector2f n1toP;
  // n1toP << x - neighbors[1](0), y - neighbors[1](1);
  // Eigen::Vector2f n2to0 = neighbors[0].head(2) - neighbors[2].head(2);
  // Eigen::Vector2f n2to1 = neighbors[1].head(2) - neighbors[2].head(2);
  // Eigen::Vector2f n2toP;
  // n2toP << x - neighbors[2](0), y - neighbors[2](1);

  // if ((n0to1*n0to2) * (n0to1*n0toP) < 0 or )

  // float area_012 =

  // float z = (neighbors[0](2) + neighbors[1](2) + neighbors[2](2))/3.0f;  // TODO: weighted distance -> Done? need to
  // test

  // std::cout << neighbors[0].head(3).transpose() << std::endl;
  // std::cout << neighbors[1].head(3).transpose() << std::endl;
  // std::cout << neighbors[2].head(3).transpose() << "\n" << std::endl;

  // float depth_max = std::max({
  //     neighbors[0](2),
  //     neighbors[1](2),
  //     neighbors[2](2)
  // });
  // float depth_min = std::min({
  //     neighbors[0](2),
  //     neighbors[1](2),
  //     neighbors[2](2)
  // });
  // if (depth_max - depth_min > 1.0)
  //     return -2.0f;

  float z = (neighbors[0](2) * neighbors[1](3) * neighbors[2](3) + neighbors[1](2) * neighbors[0](3) * neighbors[2](3) +
             neighbors[2](2) * neighbors[0](3) * neighbors[1](3)) /
            (0.0001f + neighbors[1](3) * neighbors[2](3) + neighbors[0](3) * neighbors[2](3) +
             neighbors[0](3) * neighbors[1](3));  // TODO: weighted distance -> Done? need to test
  assert(z > 0);
  return z;

}

cv::Mat PointCloudFactory::visualizeDepth(const cv::Mat& gray_image)
{
  image_with_depth = gray_image.clone();
  cv::cvtColor(image_with_depth, image_with_depth, cv::COLOR_GRAY2BGR);
  int x, y;
  float depth, depth_min = 0.1f, depth_max = 50.0f, ratio;
  for (x = 0; x < IMG_WIDTH; x += 3)
  {  // += 3 to make the visualization sparse
    for (y = 0; y < IMG_HEIGHT; y += 3)
    {
      depth = PointCloudUtil::queryDepth(static_cast<float>(x), static_cast<float>(y));
      if (depth > 0)
      {
          ratio = std::max(std::min((depth - depth_min) / (depth_max - depth_min), 1.0f), 0.0f);
          cv::circle(image_with_depth, cv::Point(x, y),  // x, y
                     1, cv::Scalar(0, 255 * ratio * 2, 255 * (1 - ratio * 2)), cv::FILLED, cv::LINE_8);

        // ratio = std::max(std::min((depth - depth_min) / (depth_max - depth_min), 1.0f), 0.0f);
        // if (ratio < 0.5)
        // {
        //   cv::circle(image_with_depth, cv::Point(x, y),  // x, y
        //              1, cv::Scalar(0, 255 * ratio * 2, 255 * (1 - ratio * 2)), cv::FILLED, cv::LINE_8);
        // }
        // else
        // {
        //   cv::circle(image_with_depth, cv::Point(x, y), 1,
        //              cv::Scalar(255 * (ratio - 0.5) * 2, 255 * (1 - (ratio - 0.5) * 2), 0), cv::FILLED, cv::LINE_8);
        // }
      }
    }
  }

  return image_with_depth;

}