#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "spdlog/spdlog.h"


int main() {
  spdlog::info("Downsampling the point cloud.");

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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->reserve(150'000);
  cloud_downsampled->reserve(150'000);

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

  /* Downsample the point cloud. */
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(0.1f,
                         0.1f,
                         0.1f);
  voxel_grid.filter(*cloud_downsampled);
  
  /* Print the number of points in the downsampled point cloud. */
  spdlog::info("Cloud({}) -> CloudDownsampled({})", cloud->size(),
                                                    cloud_downsampled->size());
  spdlog::info("LeafSize: {}, {}, {}", voxel_grid.getLeafSize().x,
                                      voxel_grid.getLeafSize().y,
                                      voxel_grid.getLeafSize().z);
  
  /* Change the color of the downsampled points. */
  for (auto& point: cloud_downsampled->points) {
    point.r = 255;
    point.g = 0;
    point.b = 0;
  }

  /* Visualize the point cloud. */
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
  viewer->addPointCloud(cloud, "cloud");

  pcl::visualization::PCLVisualizer::Ptr viewer_downsampled(new pcl::visualization::PCLVisualizer("Cloud Viewer Downsampled"));
  viewer_downsampled->addPointCloud(cloud_downsampled, "cloud_downsampled");

  while (!viewer->wasStopped() && !viewer_downsampled->wasStopped()) {
    viewer->spinOnce(100);
    viewer_downsampled->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}

