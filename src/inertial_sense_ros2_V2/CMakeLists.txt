cmake_minimum_required(VERSION 3.8)
project(inertial_sense_ros2_v2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
find_package(yaml-cpp REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GTime.msg"
  "msg/SatInfo.msg"
  "msg/GPS.msg"
  "msg/GPSInfo.msg"
  "msg/PreIntIMU.msg"
  "msg/RTKInfo.msg"
  "msg/RTKRel.msg"
  "msg/GlonassEphemeris.msg"
  "msg/GNSSEphemeris.msg"
  "msg/GNSSObservation.msg"
  "msg/GNSSObsVec.msg"
  "msg/INL2States.msg"
  "srv/FirmwareUpdate.srv"
  "srv/RefLLAUpdate.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Build InertialSenseSDK 1.12.0 as a static library
set(INERTIAL_SENSE_SDK_SOURCES
  lib/InertialSenseSDK/src/cltool.cpp
  lib/InertialSenseSDK/src/convert_ins.cpp
  lib/InertialSenseSDK/src/com_manager.c
  lib/InertialSenseSDK/src/data_sets.c
  lib/InertialSenseSDK/src/DataChunk.cpp
  lib/InertialSenseSDK/src/DataChunkSorted.cpp
  lib/InertialSenseSDK/src/DataCSV.cpp
  lib/InertialSenseSDK/src/DataJSON.cpp
  lib/InertialSenseSDK/src/DataKML.cpp
  lib/InertialSenseSDK/src/DeviceLog.cpp
  lib/InertialSenseSDK/src/DeviceLogCSV.cpp
  lib/InertialSenseSDK/src/DeviceLogJSON.cpp
  lib/InertialSenseSDK/src/DeviceLogKML.cpp
  lib/InertialSenseSDK/src/DeviceLogRaw.cpp
  lib/InertialSenseSDK/src/DeviceLogSerial.cpp
  lib/InertialSenseSDK/src/DeviceLogSorted.cpp
  lib/InertialSenseSDK/src/filters.cpp
  lib/InertialSenseSDK/src/ihex.c
  lib/InertialSenseSDK/src/InertialSense.cpp
  lib/InertialSenseSDK/src/ISClient.cpp
  lib/InertialSenseSDK/src/ISComm.c
  lib/InertialSenseSDK/src/ISCommDataBuffer.cpp
  lib/InertialSenseSDK/src/ISDataMappings.cpp
  lib/InertialSenseSDK/src/ISDisplay.cpp
  lib/InertialSenseSDK/src/ISEarth.c
  lib/InertialSenseSDK/src/ISFileManager.cpp
  lib/InertialSenseSDK/src/ISGnss.cpp
  lib/InertialSenseSDK/src/ISLogFile.cpp
  lib/InertialSenseSDK/src/ISLogger.cpp
  lib/InertialSenseSDK/src/ISLogStats.cpp
  lib/InertialSenseSDK/src/ISMatrix.c
  lib/InertialSenseSDK/src/ISPolynomial.c
  lib/InertialSenseSDK/src/ISPose.c
  lib/InertialSenseSDK/src/ISSerialPort.cpp
  lib/InertialSenseSDK/src/ISStream.cpp
  lib/InertialSenseSDK/src/ISTcpClient.cpp
  lib/InertialSenseSDK/src/ISTcpServer.cpp
  lib/InertialSenseSDK/src/ISUtilities.cpp
  lib/InertialSenseSDK/src/linked_list.c
  lib/InertialSenseSDK/src/message_stats.cpp
  lib/InertialSenseSDK/src/ring_buffer.c
  lib/InertialSenseSDK/src/serialPort.c
  lib/InertialSenseSDK/src/serialPortPlatform.c
  lib/InertialSenseSDK/src/statistics.cpp
  lib/InertialSenseSDK/src/time_conversion.c
  lib/InertialSenseSDK/src/protocol_nmea.cpp
  lib/InertialSenseSDK/src/tinystr.cpp
  lib/InertialSenseSDK/src/tinyxml.cpp
  lib/InertialSenseSDK/src/tinyxmlerror.cpp
  lib/InertialSenseSDK/src/tinyxmlparser.cpp
)

add_library(InertialSenseSDK_V2 STATIC ${INERTIAL_SENSE_SDK_SOURCES})

target_compile_options(InertialSenseSDK_V2 PRIVATE
  -Wno-format-extra-args
  -Wno-pedantic
  -Wno-unused-parameter
  -Wno-unused-variable
  -Wno-sign-compare
  -Wno-type-limits
  -fms-extensions
  -fPIC
)

target_compile_definitions(InertialSenseSDK_V2 PRIVATE
  PLATFORM_IS_LINUX
  EXCLUDE_BOOTLOADER
)

target_include_directories(InertialSenseSDK_V2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/InertialSenseSDK/src>
  $<INSTALL_INTERFACE:include>
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(InertialSenseSDK_V2 PUBLIC Threads::Threads ${LIBUSB_LIBRARIES} yaml-cpp)

# Include directories
include_directories(
  include
  lib/InertialSenseSDK/src
)

# Build the main node
add_executable(inertial_sense_node_v2
  src/inertial_sense_node.cpp
)

target_link_libraries(inertial_sense_node_v2
  InertialSenseSDK_V2
  Threads::Threads
)

ament_target_dependencies(inertial_sense_node_v2
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_compile_definitions(inertial_sense_node_v2 PRIVATE EXCLUDE_BOOTLOADER PLATFORM_IS_LINUX)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(inertial_sense_node_v2 "${cpp_typesupport_target}")

# Install
install(TARGETS
  inertial_sense_node_v2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
