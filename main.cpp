#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "core/VioManager.h"
#include "state/State.h"
#include "utils/print.h"
#include "utils/sensor_data.h"
#include "utils/opencv_yaml_parse.h"

namespace fs = std::filesystem;

struct ImuRow {
  std::int64_t stamp_ns{};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
};

static std::vector<double> split_numbers(const std::string &line) {
  std::vector<double> values;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) values.push_back(std::stod(field));
  return values;
}

static std::vector<ImuRow> load_imu(const fs::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open IMU file: " + path.string());
  std::vector<ImuRow> rows;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto v = split_numbers(line);
    if (v.size() != 17) throw std::runtime_error("Invalid xsens_imu.csv row");
    ImuRow row;
    row.stamp_ns = std::stoll(line.substr(0, line.find(',')));
    row.gyro = Eigen::Vector3d(v[8], v[9], v[10]);
    row.accel = Eigen::Vector3d(v[11], v[12], v[13]);
    rows.push_back(row);
  }
  return rows;
}

static std::vector<std::int64_t> load_stamps(const fs::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open stereo stamps: " + path.string());
  std::vector<std::int64_t> stamps;
  std::string line;
  while (std::getline(input, line)) if (!line.empty()) stamps.push_back(std::stoll(line));
  return stamps;
}

int main(int argc, char **argv) {
  if (argc < 5) {
    std::cerr << "Usage: " << argv[0]
              << " DATASET_DIR OPENVINS_CONFIG OUTPUT_CSV MAX_FRAMES(0=all)\n";
    return 2;
  }

  try {
    const fs::path root = argv[1];
    const std::string config = argv[2];
    const fs::path output_path = argv[3];
    const std::size_t max_frames = std::stoull(argv[4]);

    auto parser = std::make_shared<ov_core::YamlParser>(config);
    std::string verbosity = "INFO";
    parser->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);

    ov_msckf::VioManagerOptions options;
    options.print_and_load(parser);
    options.use_multi_threading_pubs = false;
    options.use_multi_threading_subs = false;
    if (!parser->successful()) throw std::runtime_error("OpenVINS configuration is incomplete");

    auto vio = std::make_shared<ov_msckf::VioManager>(options);
    ov_core::Printer::setPrintLevel("SILENT");
    const auto imu = load_imu(root / "sensor_data/xsens_imu.csv");
    const auto stamps = load_stamps(root / "sensor_data/stereo_stamp.csv");
    if (imu.empty() || stamps.empty()) throw std::runtime_error("Dataset has no measurements");

    std::ofstream output(output_path);
    if (!output) throw std::runtime_error("Cannot create output: " + output_path.string());
    output << "index,timestamp,x,y,z,qx,qy,qz,qw\n";
    output << std::setprecision(17);

    std::size_t imu_index = 0;
    std::size_t written = 0;
    const std::size_t frame_count = max_frames == 0 ? stamps.size() : std::min(max_frames, stamps.size());

    for (std::size_t i = 0; i < frame_count; ++i) {
      const auto stamp = stamps[i];
      while (imu_index < imu.size() && imu[imu_index].stamp_ns <= stamp) {
        ov_core::ImuData measurement;
        measurement.timestamp = static_cast<double>(imu[imu_index].stamp_ns) * 1e-9;
        measurement.wm = imu[imu_index].gyro;
        measurement.am = imu[imu_index].accel;
        vio->feed_measurement_imu(measurement);
        ++imu_index;
      }

      const fs::path left_path = root / "image/stereo_left" / (std::to_string(stamp) + ".png");
      const fs::path right_path = root / "image/stereo_right" / (std::to_string(stamp) + ".png");
      cv::Mat left = cv::imread(left_path.string(), cv::IMREAD_GRAYSCALE);
      cv::Mat right = cv::imread(right_path.string(), cv::IMREAD_GRAYSCALE);
      if (left.empty() || right.empty()) {
        std::cerr << "Skipping missing stereo pair at " << stamp << "\n";
        continue;
      }

      ov_core::CameraData camera;
      camera.timestamp = static_cast<double>(stamp) * 1e-9;
      camera.sensor_ids = {0, 1};
      camera.images = {left, right};
      camera.masks = {cv::Mat::zeros(left.size(), CV_8UC1), cv::Mat::zeros(right.size(), CV_8UC1)};
      vio->feed_measurement_camera(camera);

      if (vio->initialized()) {
        const auto state = vio->get_state();
        const auto p = state->_imu->pos();
        const auto q = state->_imu->quat();
        output << written++ << ',' << state->_timestamp << ','
               << p(0) << ',' << p(1) << ',' << p(2) << ','
               << q(0) << ',' << q(1) << ',' << q(2) << ',' << q(3) << '\n';
      }

      if ((i + 1) % 100 == 0 || i + 1 == frame_count) {
        std::cout << "Processed " << (i + 1) << '/' << frame_count
                  << " frames, poses=" << written << std::endl;
      }
    }

    std::cout << "Done. trajectory=" << output_path << " poses=" << written << std::endl;
    return written > 0 ? 0 : 3;
  } catch (const std::exception &error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 1;
  }
}

