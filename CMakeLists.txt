cmake_minimum_required(VERSION 3.16)
project(openvins_kaist_rosfree LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

set(OPENVINS_ROOT "/root/open_vins" CACHE PATH "OpenVINS source tree")
set(OPENVINS_BUILD "${OPENVINS_ROOT}/ov_msckf/build" CACHE PATH "OpenVINS build tree")

add_executable(openvins_kaist_rosfree main.cpp)
target_compile_definitions(openvins_kaist_rosfree PRIVATE ROS_AVAILABLE=0 ENABLE_ARUCO_TAGS=0)
target_include_directories(openvins_kaist_rosfree PRIVATE
  "${OPENVINS_ROOT}/ov_msckf/src"
  "${OPENVINS_ROOT}/ov_core/src"
  "${OPENVINS_ROOT}/ov_init/src"
  ${OpenCV_INCLUDE_DIRS}
)
target_link_directories(openvins_kaist_rosfree PRIVATE "${OPENVINS_BUILD}")
target_link_libraries(openvins_kaist_rosfree PRIVATE
  ov_msckf_lib
  ${OpenCV_LIBS}
  Ceres::ceres
  Boost::system Boost::filesystem Boost::thread Boost::date_time
)
set_target_properties(openvins_kaist_rosfree PROPERTIES
  BUILD_RPATH "${OPENVINS_BUILD}"
)

add_executable(kaist_stereo_cloud stereo_cloud.cpp)
target_link_libraries(kaist_stereo_cloud PRIVATE ${OpenCV_LIBS} Eigen3::Eigen)

