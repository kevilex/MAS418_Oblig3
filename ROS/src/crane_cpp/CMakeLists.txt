cmake_minimum_required(VERSION 3.8)
project(crane_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_library(
    LIBADS_LIBRARY
    NAMES AdsLib
    HINTS "../../ADS/build"
)

add_executable(talker src/publisher_member_function.cpp)
add_executable(listener src/subscriber_member_function.cpp)
add_executable(ads_node src/ads_node.cpp)


target_compile_features(ads_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(talker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(ads_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(ads_node PRIVATE "../../ADS/AdsLib")

target_include_directories(talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(talker PRIVATE "../../ADS/AdsLib")

target_link_libraries(ads_node ${LIBADS_LIBRARY} Threads::Threads)
target_link_libraries(talker ${LIBADS_LIBRARY} Threads::Threads)

ament_target_dependencies(talker rclcpp std_msgs sensor_msgs)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(ads_node rclcpp std_msgs sensor_msgs)

install(TARGETS ads_node
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
