# Define the minimum version of CMake required
cmake_minimum_required(VERSION 3.8)

# Define the package name (replace 'image_conversion' with your package name)
project(image_conversion)

# Find required packages
find_package(ament_cmake REQUIRED)         # Core build tool for ROS 2
find_package(rclcpp REQUIRED)             # ROS 2 C++ client library
find_package(sensor_msgs REQUIRED)        # ROS 2 sensor_msgs package for Image messages
find_package(std_srvs REQUIRED)           # ROS 2 std_srvs package for SetBool service
find_package(cv_bridge REQUIRED)          # ROS 2 cv_bridge package for OpenCV integration
find_package(OpenCV REQUIRED)             # OpenCV for image processing

# Define the executable node
add_executable(image_conversion_node src/image_conversion_node.cpp)

# Specify include directories
target_include_directories(image_conversion_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}                  # Include OpenCV headers
)

# Link required libraries to the executable
target_link_libraries(image_conversion_node
  ${OpenCV_LIBS}                          # OpenCV libraries
)

# Specify ROS 2 dependencies for the executable
ament_target_dependencies(image_conversion_node
  rclcpp
  sensor_msgs
  std_srvs
  cv_bridge
)

# Install the executable so ROS 2 can find it
install(TARGETS
  image_conversion_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install any additional files like launch files or configuration
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Enable ament_cmake for this package
ament_package()
