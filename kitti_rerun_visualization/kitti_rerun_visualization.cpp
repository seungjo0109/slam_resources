#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <rerun.hpp>


void LoadKittiCloud(const std::string& filename,
                    std::vector<rerun::Position3D>& points,
                    std::vector<rerun::Color>& colors) {
  std::ifstream file_kitti;
  file_kitti.open(filename, std::ios_base::binary);
  assert(file_kitti.is_open());

  while (!file_kitti.eof()) {
    // Read the point cloud from the binary file.
    float x, y, z, intensity;
    while (file_kitti.read(reinterpret_cast<char*>(&x), sizeof(float)) &&
           file_kitti.read(reinterpret_cast<char*>(&y), sizeof(float)) &&
           file_kitti.read(reinterpret_cast<char*>(&z), sizeof(float)) &&
           file_kitti.read(reinterpret_cast<char*>(&intensity), sizeof(float))) {
      points.push_back(rerun::Position3D(x, y, z));
      colors.push_back(rerun::Color(0, 255, 0));
    }
  }
}


int main() {
  /* Create a new `RecordingStream` which sends data over TCP to the viewer process. */
  const auto rec = rerun::RecordingStream("kitti_rerun_visualization");
  rec.spawn().exit_on_failure();

  /* Configure the path to the KITTI dataset. */
  std::filesystem::path kitti_path = "../kitti_velodyne";

  /* Reserve space for the points and colors. */
  std::vector<rerun::Position3D> points;
  std::vector<rerun::Color> colors;
  
  points.reserve(150'000);
  colors.reserve(150'000);

  /* Iterate over all files in the KITTI dataset. */
  int file_idx = 0;
  while (true) {
    /* Construct the file name. */
    std::ostringstream file_name;
    file_name << std::setw(10) << std::setfill('0') << file_idx << ".bin";
    std::filesystem::path file_path = kitti_path / file_name.str();

    /* Check if the file exists. */
    if (std::filesystem::exists(file_path)) {
      std::cout << "Processing file: " << file_path << std::endl;
      
      /* Clear the points and colors. */
      points.clear();
      colors.clear();

      /* Load the KITTI point cloud. */
      LoadKittiCloud(file_path, points, colors);

      /* Log the "kitti_points" entity with our data, using the `Points3D` archetype. */
      rec.set_time_sequence("frame_nr", file_idx);
      rec.log("kitti_points", rerun::Points3D(points).with_colors(colors).with_radii({0.05f}));
      file_idx++;

      /* KITTI dataset per frame is approximately 10 Hz. */
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else {
      break;
    }
  }

  return  0;
}


