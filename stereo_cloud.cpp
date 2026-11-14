#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace fs = std::filesystem;

struct Pose {
  double time{};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  // OpenVINS stores a JPL quaternion [x,y,z,w] for R_GtoI.
  // Eigen uses the Hamilton convention, whose vector part has the opposite
  // sign for the same rotation matrix.
  Eigen::Quaterniond q_g_to_i_hamilton{Eigen::Quaterniond::Identity()};
};

#pragma pack(push, 1)
struct PointXYZRGB {
  float x, y, z, rgb;
};
#pragma pack(pop)

static std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

static std::vector<Pose> load_poses(const fs::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open trajectory: " + path.string());
  std::vector<Pose> poses;
  std::string line;
  std::getline(input, line);
  while (std::getline(input, line)) {
    const auto f = split(line);
    if (f.size() != 9) continue;
    Pose p;
    p.time = std::stod(f[1]);
    p.position = Eigen::Vector3d(std::stod(f[2]), std::stod(f[3]), std::stod(f[4]));
    p.q_g_to_i_hamilton =
        Eigen::Quaterniond(std::stod(f[8]), -std::stod(f[5]), -std::stod(f[6]), -std::stod(f[7])).normalized();
    poses.push_back(p);
  }
  return poses;
}

static std::vector<std::int64_t> load_stamps(const fs::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open stereo stamps");
  std::vector<std::int64_t> stamps;
  std::string line;
  while (std::getline(input, line)) if (!line.empty()) stamps.push_back(std::stoll(line));
  return stamps;
}

static std::int64_t nearest_stamp(const std::vector<std::int64_t> &stamps, double seconds) {
  const std::int64_t target = static_cast<std::int64_t>(std::llround(seconds * 1e9));
  auto it = std::lower_bound(stamps.begin(), stamps.end(), target);
  if (it == stamps.begin()) return *it;
  if (it == stamps.end()) return stamps.back();
  return std::llabs(*it - target) < std::llabs(*(it - 1) - target) ? *it : *(it - 1);
}

static void write_pcd(const fs::path &path, const std::vector<PointXYZRGB> &points) {
  std::ofstream out(path, std::ios::binary);
  if (!out) throw std::runtime_error("Cannot create PCD: " + path.string());
  out << "# .PCD v0.7 - Point Cloud Data file format\n"
      << "VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\n"
      << "TYPE F F F F\nCOUNT 1 1 1 1\n"
      << "WIDTH " << points.size() << "\nHEIGHT 1\n"
      << "VIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << points.size() << "\nDATA binary\n";
  out.write(reinterpret_cast<const char *>(points.data()),
            static_cast<std::streamsize>(points.size() * sizeof(PointXYZRGB)));
}

int main(int argc, char **argv) {
  if (argc < 8) {
    std::cerr << "Usage: " << argv[0]
              << " DATASET TRAJECTORY OUTPUT_PCD FRAME_STRIDE PIXEL_STEP MAX_DEPTH MAX_FRAMES(0=all)\n";
    return 2;
  }
  try {
    const fs::path root = argv[1];
    const fs::path trajectory = argv[2];
    const fs::path output = argv[3];
    const int frame_stride = std::stoi(argv[4]);
    const int pixel_step = std::stoi(argv[5]);
    const double max_depth = std::stod(argv[6]);
    const std::size_t max_frames = std::stoull(argv[7]);

    cv::FileStorage left_fs((root / "calibration/left.yaml").string(), cv::FileStorage::READ);
    cv::FileStorage right_fs((root / "calibration/right.yaml").string(), cv::FileStorage::READ);
    cv::Mat k1, d1, r1, p1, k2, d2, r2, p2;
    left_fs["camera_matrix"] >> k1; left_fs["distortion_coefficients"] >> d1;
    left_fs["rectification_matrix"] >> r1; left_fs["projection_matrix"] >> p1;
    right_fs["camera_matrix"] >> k2; right_fs["distortion_coefficients"] >> d2;
    right_fs["rectification_matrix"] >> r2; right_fs["projection_matrix"] >> p2;
    if (k1.empty() || p2.empty()) throw std::runtime_error("Invalid stereo calibration");

    const cv::Size image_size(1280, 560);
    cv::Mat map1x, map1y, map2x, map2y;
    cv::initUndistortRectifyMap(k1, d1, r1, p1(cv::Rect(0, 0, 3, 3)), image_size, CV_32FC1, map1x, map1y);
    cv::initUndistortRectifyMap(k2, d2, r2, p2(cv::Rect(0, 0, 3, 3)), image_size, CV_32FC1, map2x, map2y);
    const double fx = p1.at<double>(0, 0);
    const double fy = p1.at<double>(1, 1);
    const double cx = p1.at<double>(0, 2);
    const double cy = p1.at<double>(1, 2);
    const double baseline = std::abs(p2.at<double>(0, 3) / p2.at<double>(0, 0));
    Eigen::Matrix3d r_rect_to_cam;
    cv::cv2eigen(r1.t(), r_rect_to_cam);

    const Eigen::Matrix3d r_c_to_i = (Eigen::Matrix3d() <<
      -0.00681, -0.01532, 0.99987,
      -0.99998, 0.00033, -0.00680,
      -0.00023, -0.99988, -0.01532).finished();
    const Eigen::Vector3d p_c_in_i(1.71239, 0.24740, -0.11589);

    auto matcher = cv::StereoSGBM::create(0, 256, 5);
    matcher->setP1(8 * 5 * 5);
    matcher->setP2(32 * 5 * 5);
    matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
    matcher->setUniquenessRatio(10);
    matcher->setSpeckleWindowSize(100);
    matcher->setSpeckleRange(2);
    matcher->setDisp12MaxDiff(1);

    const auto poses = load_poses(trajectory);
    const auto stamps = load_stamps(root / "sensor_data/stereo_stamp.csv");
    const std::size_t available = max_frames == 0 ? poses.size() : std::min(max_frames, poses.size());
    std::vector<PointXYZRGB> points;
    points.reserve((available / frame_stride + 1) * 30000);

    std::size_t used_frames = 0;
    for (std::size_t i = 0; i < available; i += frame_stride) {
      const auto stamp = nearest_stamp(stamps, poses[i].time);
      cv::Mat left = cv::imread((root / "image/stereo_left" / (std::to_string(stamp) + ".png")).string(), cv::IMREAD_GRAYSCALE);
      cv::Mat right = cv::imread((root / "image/stereo_right" / (std::to_string(stamp) + ".png")).string(), cv::IMREAD_GRAYSCALE);
      if (left.empty() || right.empty()) continue;
      cv::Mat left_rect, right_rect, disparity16;
      cv::remap(left, left_rect, map1x, map1y, cv::INTER_LINEAR);
      cv::remap(right, right_rect, map2x, map2y, cv::INTER_LINEAR);
      matcher->compute(left_rect, right_rect, disparity16);

      const Eigen::Matrix3d r_i_to_g = poses[i].q_g_to_i_hamilton.toRotationMatrix().transpose();
      for (int v = 0; v < disparity16.rows; v += pixel_step) {
        const auto *disp = disparity16.ptr<std::int16_t>(v);
        const auto *gray = left_rect.ptr<std::uint8_t>(v);
        for (int u = 0; u < disparity16.cols; u += pixel_step) {
          const double d = static_cast<double>(disp[u]) / 16.0;
          if (d <= 1.0) continue;
          const double z = fx * baseline / d;
          if (!std::isfinite(z) || z < 1.0 || z > max_depth) continue;
          const Eigen::Vector3d p_rect((u - cx) * z / fx, (v - cy) * z / fy, z);
          const Eigen::Vector3d p_c = r_rect_to_cam * p_rect;
          const Eigen::Vector3d p_g = r_i_to_g * (r_c_to_i * p_c + p_c_in_i) + poses[i].position;
          const std::uint32_t rgb_u = (static_cast<std::uint32_t>(gray[u]) << 16) |
                                      (static_cast<std::uint32_t>(gray[u]) << 8) | gray[u];
          float rgb_f;
          std::memcpy(&rgb_f, &rgb_u, sizeof(float));
          points.push_back({static_cast<float>(p_g.x()), static_cast<float>(p_g.y()),
                            static_cast<float>(p_g.z()), rgb_f});
        }
      }
      ++used_frames;
      if (used_frames % 50 == 0) {
        std::cout << "Stereo frames=" << used_frames << " points=" << points.size() << std::endl;
      }
    }

    write_pcd(output, points);
    std::cout << "Done. frames=" << used_frames << " points=" << points.size()
              << " output=" << output << std::endl;
    return points.empty() ? 3 : 0;
  } catch (const std::exception &e) {
    std::cerr << "ERROR: " << e.what() << '\n';
    return 1;
  }
}

