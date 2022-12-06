cmake_minimum_required(VERSION 3.8)
project(dsso_ros)

CMAKE_MINIMUM_REQUIRED(VERSION 3.14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra)
endif()

set(DSSO_PATH /home/tp/src/dsso)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

#find_package(SuiteParse REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED COMPONENTS system thread)
include(cmake/ConfigExtras.cmake) # See https://gist.github.com/tylerjw/ce3bcb94b3df7996e450dec18ee54cde
find_package(Pangolin REQUIRED)
message("-- FOUND Pangolin version ${Pangolin_VERSION}")
find_library(DSSO_LIBRARY dsso ${DSSO_PATH}/build/Debug/lib)
message("-- FOUND dsso lib: ${DSSO_LIBRARY}")

find_package(OpenCV REQUIRED)
message("-- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

set(CMAKE_CXX_FLAGS
        "${SSE_FLAGS} -O2 -g -std=c++17 -march=native -fno-omit-frame-pointer"
)


add_executable(dsso_ros src/dsso_ros.cpp)
target_link_libraries(dsso_ros ${Boost_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${DSSO_LIBRARY})
ament_target_dependencies(dsso_ros rclcpp sensor_msgs geometry_msgs cv_bridge)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${DSSO_PATH}/src
  ${DSSO_PATH}/thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
)

install(TARGETS
  dsso_ros
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS cmake/ConfigExtras.cmake)
