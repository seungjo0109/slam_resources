#include <filesystem>
#include <thread>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "spdlog/spdlog.h"


int main() {
  spdlog::info("Filtering the point cloud using a passthrough filter.");

  /* Check if the file exists. */
  std::filesystem::path kitti_path = "../kitti_velodyne";
  std::filesystem::path file_path = kitti_path / "0000000000.bin";
  assert(std::filesystem::exists(file_path));

  /* Open the file. */
  std::ifstream file_kitti;
  file_kitti.open(file_path, std::ios_base::binary);
  assert(file_kitti.is_open());

  /* Read the point cloud from the binary file. */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr negative_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr positive_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->reserve(150'000);
  negative_filtered->reserve(150'000);
  positive_filtered->reserve(150'000);

  float x, y, z, intensity;
  while (file_kitti.read(reinterpret_cast<char*>(&x), sizeof(float)) &&
         file_kitti.read(reinterpret_cast<char*>(&y), sizeof(float)) &&
         file_kitti.read(reinterpret_cast<char*>(&z), sizeof(float)) &&
         file_kitti.read(reinterpret_cast<char*>(&intensity), sizeof(float))) {
    pcl::PointXYZRGB point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.r = 0;
    point.g = 255;
    point.b = 0;
    cloud->push_back(point);
  }

  /* PassThrough filter. */
  pcl::PassThrough<pcl::PointXYZRGB> pass_through;
  pass_through.setInputCloud(cloud);
  pass_through.setFilterFieldName("x");
  pass_through.setFilterLimits(0.0f, 10.0f);

  /* Filter the point cloud. */
  pass_through.setNegative(true);
  pass_through.filter(*negative_filtered);
  pass_through.setNegative(false);
  pass_through.filter(*positive_filtered);

  /* Print the number of points in the filtered point cloud. */
  spdlog::info("Cloud({}) -> CloudFiltered({})", cloud->size(),
                                                 negative_filtered->size());

  /* Change the color of the filtered points. */
  for (auto& point: negative_filtered->points) {
    point.r = 255;
    point.g = 0;
    point.b = 0;
  }
  for (auto& point: positive_filtered->points) {
    point.r = 0;
    point.g = 0;
    point.b = 255;
  }

  /* Visualize the point cloud. */
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
  pcl::visualization::PCLVisualizer::Ptr viewer_filtered(new pcl::visualization::PCLVisualizer("Filtered Cloud"));

  viewer->addPointCloud(cloud, "Cloud");
  viewer_filtered->addPointCloud(negative_filtered, "Negative Filtered");
  viewer_filtered->addPointCloud(positive_filtered, "Positive Filtered");

  while (!viewer->wasStopped() && !viewer_filtered->wasStopped()) {
    viewer->spinOnce(100);
    viewer_filtered->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}


