#pragma once

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>

#include "InertialSense.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "inertial_sense_ros2_v2/msg/g_time.hpp"
#include "inertial_sense_ros2_v2/msg/gps.hpp"
#include "inertial_sense_ros2_v2/msg/gps_info.hpp"
#include "inertial_sense_ros2_v2/msg/pre_int_imu.hpp"
#include "inertial_sense_ros2_v2/msg/rtk_info.hpp"
#include "inertial_sense_ros2_v2/msg/rtk_rel.hpp"
#include "inertial_sense_ros2_v2/msg/gnss_ephemeris.hpp"
#include "inertial_sense_ros2_v2/msg/glonass_ephemeris.hpp"
#include "inertial_sense_ros2_v2/msg/gnss_observation.hpp"
#include "inertial_sense_ros2_v2/msg/gnss_obs_vec.hpp"
#include "inertial_sense_ros2_v2/msg/inl2_states.hpp"
#include "inertial_sense_ros2_v2/srv/firmware_update.hpp"
#include "inertial_sense_ros2_v2/srv/ref_lla_update.hpp"

#define GPS_UNIX_OFFSET   315964800   // GPS epoch (1980-01-06) minus UNIX epoch (1970-01-01) in seconds
#define LEAP_SECONDS      18          // GPS does not have leap seconds (as of 2017)
#define UNIX_TO_GPS_OFFSET (GPS_UNIX_OFFSET - LEAP_SECONDS)

class InertialSenseROS : public rclcpp::Node
{
public:
  InertialSenseROS();
  ~InertialSenseROS() = default;

  void callback(p_data_t* data);
  void update();

private:
  // Initialization
  void connect();
  void set_navigation_dt_ms();
  void configure_parameters();
  void configure_rtk();
  void configure_data_streams();
  void start_log();

  template<typename T>
  void set_vector_flash_config(const std::string& param_name, uint32_t size, uint32_t offset);

  template<typename T>
  void set_flash_config(const std::string& param_name, uint32_t offset, T def);

  void reset_device();

  // Serial / device state
  std::string port_;
  int baudrate_;
  bool initialized_;
  bool log_enabled_;
  std::string frame_id_;

  // ROS2 stream handle
  struct ros_stream_t
  {
    bool enabled = false;
    rclcpp::PublisherBase::SharedPtr pub;
    rclcpp::PublisherBase::SharedPtr pub2;
  };

  // Helper to create typed publisher and store as base pointer
  template<typename T>
  typename rclcpp::Publisher<T>::SharedPtr make_publisher(const std::string& topic, size_t qos)
  {
    return this->create_publisher<T>(topic, qos);
  }

  // Typed publisher helpers to avoid repeated casts
  template<typename T>
  void publish(ros_stream_t& stream, const T& msg)
  {
    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(stream.pub);
    if (pub) pub->publish(msg);
  }

  template<typename T>
  void publish2(ros_stream_t& stream, const T& msg)
  {
    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(stream.pub2);
    if (pub) pub->publish(msg);
  }

  // INS
  ros_stream_t INS_;
  void INS1_callback(const ins_1_t* const msg);
  void INS2_callback(const ins_2_t* const msg);

  // IMU (SDK 1.12: DID_IMU → imu_t)
  ros_stream_t IMU_;
  void IMU_callback(const imu_t* const msg);

  // GPS
  ros_stream_t GPS_;
  ros_stream_t GPS_obs_;
  ros_stream_t GPS_eph_;
  void GPS_pos_callback(const gps_pos_t* const msg);
  void GPS_vel_callback(const gps_vel_t* const msg);
  void GPS_raw_callback(const gps_raw_t* const msg);
  void GPS_obs_callback(const obsd_t* const msg, int nObs);
  void GPS_eph_callback(const eph_t* const msg);
  void GPS_geph_callback(const geph_t* const msg);
  void GPS_obs_bundle_timer_callback();

  inertial_sense_ros2_v2::msg::GNSSObsVec obs_Vec_;
  rclcpp::TimerBase::SharedPtr obs_bundle_timer_;
  rclcpp::Time last_obs_time_;

  ros_stream_t GPS_info_;
  void GPS_info_callback(const gps_sat_t* const msg);

  // Magnetometer
  ros_stream_t mag_;
  void mag_callback(const magnetometer_t* const msg);

  // Barometer
  ros_stream_t baro_;
  void baro_callback(const barometer_t* const msg);

  // Pre-integrated IMU (SDK 1.12: DID_PIMU → pimu_t)
  ros_stream_t dt_vel_;
  void preint_IMU_callback(const pimu_t* const msg);

  // Strobe
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr strobe_pub_;
  void strobe_in_time_callback(const strobe_in_time_t* const msg);

  // RTK
  typedef enum { RTK_NONE, RTK_ROVER, RTK_BASE, DUAL_GNSS } rtk_state_t;
  rtk_state_t RTK_state_ = RTK_NONE;
  ros_stream_t RTK_;
  void RTK_Misc_callback(const gps_rtk_misc_t* const msg);
  void RTK_Rel_callback(const gps_rtk_rel_t* const msg);

  // Wheel encoder
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_enc_sub_;
  void wheel_enc_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mag_cal_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr multi_mag_cal_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr refLLA_set_current_srv_;
  rclcpp::Service<inertial_sense_ros2_v2::srv::RefLLAUpdate>::SharedPtr refLLA_set_value_srv_;
  rclcpp::Service<inertial_sense_ros2_v2::srv::FirmwareUpdate>::SharedPtr firmware_update_srv_;

  bool set_current_position_as_refLLA(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  bool set_refLLA_to_value(
    const inertial_sense_ros2_v2::srv::RefLLAUpdate::Request::SharedPtr req,
    inertial_sense_ros2_v2::srv::RefLLAUpdate::Response::SharedPtr res);

  bool perform_mag_cal_srv_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  bool perform_multi_mag_cal_srv_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  bool update_firmware_srv_callback(
    const inertial_sense_ros2_v2::srv::FirmwareUpdate::Request::SharedPtr req,
    inertial_sense_ros2_v2::srv::FirmwareUpdate::Response::SharedPtr res);

  void publishGPS();

  // Time utilities
  rclcpp::Time ros_time_from_week_and_tow(uint32_t week, double timeOfWeek);
  rclcpp::Time ros_time_from_start_time(double time);
  rclcpp::Time ros_time_from_tow(double tow);
  double tow_from_ros_time(const rclcpp::Time& rt);
  rclcpp::Time ros_time_from_gtime(uint64_t sec, double subsec);

  // Time sync state
  double GPS_towOffset_ = 0.0;
  uint64_t GPS_week_ = 0;
  double INS_local_offset_ = 0.0;
  bool got_first_message_ = false;

  // Cached messages
  double lla_[3] = {0, 0, 0};
  sensor_msgs::msg::Imu imu1_msg_;
  nav_msgs::msg::Odometry odom_msg_;
  inertial_sense_ros2_v2::msg::GPS gps_msg_;
  geometry_msgs::msg::Vector3Stamped gps_velEcef_;
  inertial_sense_ros2_v2::msg::GPSInfo gps_info_msg_;

  // SDK connection
  InertialSense IS_;
};
