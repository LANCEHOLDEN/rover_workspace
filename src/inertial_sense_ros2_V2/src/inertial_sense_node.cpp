#include "inertial_sense_ros2_v2/inertial_sense_node.h"
#include <unistd.h>

// Macro to register a DID callback with the SDK
#define SET_CALLBACK(DID, __type, __cb_fun) \
  IS_.BroadcastBinaryData(DID, 1, \
    [this](InertialSense* /*i*/, p_data_t* data, int /*pHandle*/) \
    { \
      this->__cb_fun(reinterpret_cast<__type*>(data->buf)); \
    })

// ============================================================
//  Constructor
// ============================================================
InertialSenseROS::InertialSenseROS()
: rclcpp::Node("inertial_sense_node"), initialized_(false), log_enabled_(false)
{
  connect();
  set_navigation_dt_ms();

  // Services
  refLLA_set_current_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "set_refLLA_current",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
           std_srvs::srv::Trigger::Response::SharedPtr res)
    { set_current_position_as_refLLA(req, res); });

  refLLA_set_value_srv_ = this->create_service<inertial_sense_ros2_v2::srv::RefLLAUpdate>(
    "set_refLLA_value",
    [this](const inertial_sense_ros2_v2::srv::RefLLAUpdate::Request::SharedPtr req,
           inertial_sense_ros2_v2::srv::RefLLAUpdate::Response::SharedPtr res)
    { set_refLLA_to_value(req, res); });

  mag_cal_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "single_axis_mag_cal",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
           std_srvs::srv::Trigger::Response::SharedPtr res)
    { perform_mag_cal_srv_callback(req, res); });

  multi_mag_cal_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "multi_axis_mag_cal",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
           std_srvs::srv::Trigger::Response::SharedPtr res)
    { perform_multi_mag_cal_srv_callback(req, res); });

  firmware_update_srv_ = this->create_service<inertial_sense_ros2_v2::srv::FirmwareUpdate>(
    "firmware_update",
    [this](const inertial_sense_ros2_v2::srv::FirmwareUpdate::Request::SharedPtr req,
           inertial_sense_ros2_v2::srv::FirmwareUpdate::Response::SharedPtr res)
    { update_firmware_srv_callback(req, res); });

  wheel_enc_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 20,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg)
    { wheel_enc_callback(msg); });

  IS_.StopBroadcasts();
  configure_parameters();
  configure_data_streams();

  log_enabled_ = this->declare_parameter<bool>("enable_log", false);
  if (log_enabled_)
    start_log();

  configure_rtk();
  initialized_ = true;
}

// ============================================================
//  connect
// ============================================================
void InertialSenseROS::connect()
{
  port_     = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
  baudrate_ = this->declare_parameter<int>("baudrate", 921600);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "body");

  RCLCPP_INFO(this->get_logger(), "Connecting to serial port \"%s\" at %d baud",
              port_.c_str(), baudrate_);

  if (!IS_.Open(port_.c_str(), baudrate_))
  {
    RCLCPP_FATAL(this->get_logger(),
                 "Unable to open serial port \"%s\" at %d baud", port_.c_str(), baudrate_);
    rclcpp::shutdown();
    return;
  }

  nvm_flash_cfg_t flash;
  IS_.GetFlashConfig(flash);
  RCLCPP_INFO(this->get_logger(), "Connected to uINS on \"%s\" at %d baud",
              port_.c_str(), baudrate_);
}

// ============================================================
//  set_navigation_dt_ms
// ============================================================
void InertialSenseROS::set_navigation_dt_ms()
{
  int nav_dt_ms = this->declare_parameter<int>("navigation_dt_ms", 10);

  nvm_flash_cfg_t flash;
  if (IS_.GetFlashConfig(flash))
  {
    if (nav_dt_ms != static_cast<int>(flash.startupNavDtMs))
    {
      uint32_t data = static_cast<uint32_t>(nav_dt_ms);
      IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&data),
                   sizeof(uint32_t), offsetof(nvm_flash_cfg_t, startupNavDtMs));
      RCLCPP_INFO(this->get_logger(),
                  "Navigation rate changed from %dms to %dms, resetting uINS",
                  flash.startupNavDtMs, nav_dt_ms);
      sleep(3);
      reset_device();
    }
  }
}

// ============================================================
//  configure_parameters  (writes to device flash)
// ============================================================
void InertialSenseROS::configure_parameters()
{
  set_vector_flash_config<float>("INS_rpy",      3, offsetof(nvm_flash_cfg_t, insRotation));
  set_vector_flash_config<float>("INS_xyz",      3, offsetof(nvm_flash_cfg_t, insOffset));
  set_vector_flash_config<float>("GPS_ant1_xyz", 3, offsetof(nvm_flash_cfg_t, gps1AntOffset));
  set_vector_flash_config<float>("GPS_ant2_xyz", 3, offsetof(nvm_flash_cfg_t, gps2AntOffset));
  set_vector_flash_config<double>("GPS_ref_lla", 3, offsetof(nvm_flash_cfg_t, refLla));

  // Note: magInclination was removed in SDK 1.12 – only declination remains
  set_flash_config<float>("declination",   offsetof(nvm_flash_cfg_t, magDeclination), 0.20007290992f);
  set_flash_config<int>("dynamic_model",   offsetof(nvm_flash_cfg_t, insDynModel),    8);
  set_flash_config<int>("ser1_baud_rate",  offsetof(nvm_flash_cfg_t, ser1BaudRate),   921600);
}

template<typename T>
void InertialSenseROS::set_vector_flash_config(const std::string& param_name,
                                                uint32_t size, uint32_t offset)
{
  std::vector<double> tmp(size, 0.0);
  if (this->has_parameter(param_name))
    tmp = this->get_parameter(param_name).as_double_array();
  else
    this->declare_parameter<std::vector<double>>(param_name, tmp);

  std::vector<T> v(size);
  for (uint32_t i = 0; i < size; ++i)
    v[i] = static_cast<T>(tmp[i]);

  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(v.data()),
               sizeof(T) * size, offset);
}

template<typename T>
void InertialSenseROS::set_flash_config(const std::string& param_name,
                                         uint32_t offset, T def)
{
  T tmp;
  if (this->has_parameter(param_name))
    tmp = static_cast<T>(this->get_parameter(param_name).as_double());
  else
    tmp = def;

  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&tmp), sizeof(T), offset);
}

// ============================================================
//  configure_data_streams
// ============================================================
void InertialSenseROS::configure_data_streams()
{
  // GPS position/velocity always needed for time sync
  SET_CALLBACK(DID_GPS1_POS, gps_pos_t, GPS_pos_callback);
  SET_CALLBACK(DID_GPS1_VEL, gps_vel_t, GPS_vel_callback);
  SET_CALLBACK(DID_STROBE_IN_TIME, strobe_in_time_t, strobe_in_time_callback);

  // INS
  INS_.enabled = this->declare_parameter<bool>("stream_INS", true);
  if (INS_.enabled)
  {
    INS_.pub = this->create_publisher<nav_msgs::msg::Odometry>("ins", 1);
    SET_CALLBACK(DID_INS_1, ins_1_t, INS1_callback);
    SET_CALLBACK(DID_INS_2, ins_2_t, INS2_callback);
    // SDK 1.12: DID_IMU replaces DID_DUAL_IMU; imu_t replaces dual_imu_t
    SET_CALLBACK(DID_IMU, imu_t, IMU_callback);
  }

  // IMU (separate stream)
  IMU_.enabled = this->declare_parameter<bool>("stream_IMU", false);
  if (IMU_.enabled)
  {
    IMU_.pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    if (!INS_.enabled)
    {
      SET_CALLBACK(DID_INS_1, ins_1_t, INS1_callback);
      SET_CALLBACK(DID_INS_2, ins_2_t, INS2_callback);
      SET_CALLBACK(DID_IMU, imu_t, IMU_callback);
    }
  }

  // GPS publish
  GPS_.enabled = this->declare_parameter<bool>("stream_GPS", false);
  if (GPS_.enabled)
    GPS_.pub = this->create_publisher<inertial_sense_ros2_v2::msg::GPS>("gps", 1);

  // GPS raw
  GPS_obs_.enabled = this->declare_parameter<bool>("stream_GPS_raw", false);
  GPS_eph_.enabled = GPS_obs_.enabled;
  if (GPS_obs_.enabled)
  {
    GPS_obs_.pub  = this->create_publisher<inertial_sense_ros2_v2::msg::GNSSObsVec>("gps/obs", 50);
    GPS_eph_.pub  = this->create_publisher<inertial_sense_ros2_v2::msg::GNSSEphemeris>("gps/eph", 50);
    GPS_eph_.pub2 = this->create_publisher<inertial_sense_ros2_v2::msg::GlonassEphemeris>("gps/geph", 50);
    SET_CALLBACK(DID_GPS1_RAW,     gps_raw_t, GPS_raw_callback);
    SET_CALLBACK(DID_GPS_BASE_RAW, gps_raw_t, GPS_raw_callback);
    SET_CALLBACK(DID_GPS2_RAW,     gps_raw_t, GPS_raw_callback);
    obs_bundle_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      [this]() { GPS_obs_bundle_timer_callback(); });
  }

  // GPS info
  GPS_info_.enabled = this->declare_parameter<bool>("stream_GPS_info", false);
  if (GPS_info_.enabled)
  {
    GPS_info_.pub = this->create_publisher<inertial_sense_ros2_v2::msg::GPSInfo>("gps/info", 1);
    SET_CALLBACK(DID_GPS1_SAT, gps_sat_t, GPS_info_callback);
  }

  // Magnetometer
  mag_.enabled = this->declare_parameter<bool>("stream_mag", false);
  if (mag_.enabled)
  {
    mag_.pub = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 1);
    SET_CALLBACK(DID_MAGNETOMETER, magnetometer_t, mag_callback);
  }

  // Barometer
  baro_.enabled = this->declare_parameter<bool>("stream_baro", false);
  if (baro_.enabled)
  {
    baro_.pub = this->create_publisher<sensor_msgs::msg::FluidPressure>("baro", 1);
    SET_CALLBACK(DID_BAROMETER, barometer_t, baro_callback);
  }

  // Pre-integrated IMU — SDK 1.12: DID_PIMU, pimu_t
  dt_vel_.enabled = this->declare_parameter<bool>("stream_preint_IMU", false);
  if (dt_vel_.enabled)
  {
    dt_vel_.pub = this->create_publisher<inertial_sense_ros2_v2::msg::PreIntIMU>("preint_imu", 1);
    SET_CALLBACK(DID_PIMU, pimu_t, preint_IMU_callback);
  }
}

// ============================================================
//  configure_rtk
// ============================================================
void InertialSenseROS::configure_rtk()
{
  bool RTK_rover  = this->declare_parameter<bool>("RTK_rover",  false);
  bool RTK_base   = this->declare_parameter<bool>("RTK_base",   false);
  bool dual_GNSS  = this->declare_parameter<bool>("dual_GNSS",  false);

  std::string RTK_server_IP    = this->declare_parameter<std::string>("RTK_server_IP",       "127.0.0.1");
  int         RTK_server_port  = this->declare_parameter<int>        ("RTK_server_port",      7777);
  std::string RTK_correction   = this->declare_parameter<std::string>("RTK_correction_type",  "RTCM3");
  std::string RTK_mountpoint   = this->declare_parameter<std::string>("RTK_mountpoint",       "");
  std::string RTK_username     = this->declare_parameter<std::string>("RTK_username",         "");
  std::string RTK_password     = this->declare_parameter<std::string>("RTK_password",         "");

  if (RTK_rover && RTK_base)
    RCLCPP_ERROR(this->get_logger(), "Cannot be both RTK rover and base — defaulting to rover");
  if (RTK_rover && dual_GNSS)
    RCLCPP_ERROR(this->get_logger(), "Cannot be RTK rover and dual GNSS simultaneously — defaulting to dual GNSS");

  uint32_t RTKCfgBits = 0;

  if (dual_GNSS)
  {
    RTK_rover = false;
    RCLCPP_INFO(this->get_logger(), "InertialSense: Configured as dual GNSS (compassing)");
    RTK_state_ = DUAL_GNSS;
    // SDK 1.12: RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING replaces RTK_CFG_BITS_COMPASSING
    RTKCfgBits |= RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING;
    SET_CALLBACK(DID_GPS2_RTK_CMP_MISC, gps_rtk_misc_t, RTK_Misc_callback);
    SET_CALLBACK(DID_GPS2_RTK_CMP_REL,  gps_rtk_rel_t,  RTK_Rel_callback);
    RTK_.enabled = true;
    RTK_.pub  = this->create_publisher<inertial_sense_ros2_v2::msg::RTKInfo>("RTK/info", 10);
    RTK_.pub2 = this->create_publisher<inertial_sense_ros2_v2::msg::RTKRel>("RTK/rel",  10);
  }

  if (RTK_rover)
  {
    RTK_base = false;
    // SDK 1.12 format: "TCP:RTCM3:host:port" (4 parts required by ISClient)
    std::string conn = "TCP:" + RTK_correction + ":" + RTK_server_IP + ":" +
                       std::to_string(RTK_server_port);
    RCLCPP_INFO(this->get_logger(), "InertialSense: Configured as RTK Rover → %s", conn.c_str());
    RTK_state_ = RTK_ROVER;
    // SDK 1.12: RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING replaces RTK_CFG_BITS_GPS1_RTK_ROVER
    RTKCfgBits |= RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING;

    // SDK 1.12: OpenConnectionToServer replaces OpenServerConnection
    if (IS_.OpenConnectionToServer(conn))
      RCLCPP_INFO(this->get_logger(), "Connected to RTK server: %s", conn.c_str());
    else
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to RTK server: %s", conn.c_str());

    SET_CALLBACK(DID_GPS1_RTK_POS_MISC, gps_rtk_misc_t, RTK_Misc_callback);
    SET_CALLBACK(DID_GPS1_RTK_POS_REL,  gps_rtk_rel_t,  RTK_Rel_callback);
    RTK_.enabled = true;
    RTK_.pub  = this->create_publisher<inertial_sense_ros2_v2::msg::RTKInfo>("RTK/info", 10);
    RTK_.pub2 = this->create_publisher<inertial_sense_ros2_v2::msg::RTKRel>("RTK/rel",  10);
  }
  else if (RTK_base)
  {
    std::string conn = RTK_server_IP + ":" + std::to_string(RTK_server_port);
    RTK_.enabled = true;
    RCLCPP_INFO(this->get_logger(), "InertialSense: Configured as RTK Base");
    RTK_state_ = RTK_BASE;
    RTKCfgBits |= RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0;

    if (IS_.CreateHost(conn))
    {
      RCLCPP_INFO(this->get_logger(), "Created RTK base server: %s", conn.c_str());
      initialized_ = true;
      return;
    }
    else
      RCLCPP_ERROR(this->get_logger(), "Failed to create RTK base at %s", conn.c_str());
  }

  // Read current flash config and only reset if RTKCfgBits actually changed.
  // This mirrors set_navigation_dt_ms — avoids an infinite reset loop on respawn.
  nvm_flash_cfg_t flash;
  if (IS_.GetFlashConfig(flash) && flash.RTKCfgBits != RTKCfgBits)
  {
    RCLCPP_INFO(this->get_logger(),
                "RTKCfgBits changed 0x%08X → 0x%08X, writing flash and resetting uINS",
                flash.RTKCfgBits, RTKCfgBits);
    IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&RTKCfgBits),
                 sizeof(RTKCfgBits), offsetof(nvm_flash_cfg_t, RTKCfgBits));
    sleep(3);
    reset_device();
  }
  else
  {
    IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&RTKCfgBits),
                 sizeof(RTKCfgBits), offsetof(nvm_flash_cfg_t, RTKCfgBits));
    RCLCPP_INFO(this->get_logger(),
                "RTKCfgBits unchanged (0x%08X) — no reset needed", RTKCfgBits);
  }
}

// ============================================================
//  start_log
// ============================================================
void InertialSenseROS::start_log()
{
  std::string filename = cISLogger::CreateCurrentTimestamp();
  RCLCPP_INFO(this->get_logger(), "Creating log in %s", filename.c_str());
  IS_.SetLoggerEnabled(true, filename, cISLogger::LOGTYPE_DAT, RMC_PRESET_PPD_BITS);
}

// ============================================================
//  update  (called from main loop)
// ============================================================
void InertialSenseROS::update()
{
  IS_.Update();
}

// ============================================================
//  reset_device
// ============================================================
void InertialSenseROS::reset_device()
{
  // SDK 1.12: DID_CONFIG/config_t replaced by DID_SYS_CMD/system_command_t
  system_command_t reset_command;
  reset_command.command    = 99;  // 99 = software reset
  reset_command.invCommand = ~reset_command.command;
  IS_.SendData(DID_SYS_CMD, reinterpret_cast<uint8_t*>(&reset_command), sizeof(system_command_t), 0);
  sleep(1);
}

// ============================================================
//  INS callbacks
// ============================================================
void InertialSenseROS::INS1_callback(const ins_1_t* const msg)
{
  odom_msg_.header.frame_id = frame_id_;
  odom_msg_.pose.pose.position.x = msg->ned[0];
  odom_msg_.pose.pose.position.y = msg->ned[1];
  odom_msg_.pose.pose.position.z = msg->ned[2];
}

void InertialSenseROS::INS2_callback(const ins_2_t* const msg)
{
  odom_msg_.header.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeek);
  odom_msg_.header.frame_id = frame_id_;

  odom_msg_.pose.pose.orientation.w = msg->qn2b[0];
  odom_msg_.pose.pose.orientation.x = msg->qn2b[1];
  odom_msg_.pose.pose.orientation.y = msg->qn2b[2];
  odom_msg_.pose.pose.orientation.z = msg->qn2b[3];

  odom_msg_.twist.twist.linear.x = msg->uvw[0];
  odom_msg_.twist.twist.linear.y = msg->uvw[1];
  odom_msg_.twist.twist.linear.z = msg->uvw[2];

  lla_[0] = msg->lla[0];
  lla_[1] = msg->lla[1];
  lla_[2] = msg->lla[2];

  odom_msg_.twist.twist.angular.x = imu1_msg_.angular_velocity.x;
  odom_msg_.twist.twist.angular.y = imu1_msg_.angular_velocity.y;
  odom_msg_.twist.twist.angular.z = imu1_msg_.angular_velocity.z;

  if (INS_.enabled)
  {
    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<nav_msgs::msg::Odometry>>(INS_.pub);
    if (pub) pub->publish(odom_msg_);
  }
}

// ============================================================
//  IMU callback  — SDK 1.12: imu_t (was dual_imu_t)
// ============================================================
void InertialSenseROS::IMU_callback(const imu_t* const msg)
{
  imu1_msg_.header.stamp    = ros_time_from_start_time(msg->time);
  imu1_msg_.header.frame_id = frame_id_;

  imu1_msg_.angular_velocity.x    = msg->I.pqr[0];
  imu1_msg_.angular_velocity.y    = msg->I.pqr[1];
  imu1_msg_.angular_velocity.z    = msg->I.pqr[2];
  imu1_msg_.linear_acceleration.x = msg->I.acc[0];
  imu1_msg_.linear_acceleration.y = msg->I.acc[1];
  imu1_msg_.linear_acceleration.z = msg->I.acc[2];

  if (IMU_.enabled)
  {
    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Imu>>(IMU_.pub);
    if (pub) pub->publish(imu1_msg_);
  }
}

// ============================================================
//  GPS callbacks
// ============================================================
void InertialSenseROS::GPS_pos_callback(const gps_pos_t* const msg)
{
  GPS_week_      = msg->week;
  GPS_towOffset_ = msg->towOffset;

  if (GPS_.enabled)
  {
    gps_msg_.header.stamp    = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs / 1e3);
    gps_msg_.header.frame_id = frame_id_;
    gps_msg_.fix_type  = msg->status & GPS_STATUS_FIX_MASK;
    gps_msg_.num_sat   = static_cast<int8_t>(msg->status & GPS_STATUS_NUM_SATS_USED_MASK);
    gps_msg_.cno       = static_cast<int32_t>(msg->cnoMean);
    gps_msg_.latitude  = msg->lla[0];
    gps_msg_.longitude = msg->lla[1];
    gps_msg_.altitude  = msg->lla[2];
    gps_msg_.pos_ecef.x = msg->ecef[0];
    gps_msg_.pos_ecef.y = msg->ecef[1];
    gps_msg_.pos_ecef.z = msg->ecef[2];
    gps_msg_.h_msl  = msg->hMSL;
    gps_msg_.h_acc  = msg->hAcc;
    gps_msg_.v_acc  = msg->vAcc;
    gps_msg_.p_dop  = msg->pDop;
    publishGPS();
  }
}

void InertialSenseROS::GPS_vel_callback(const gps_vel_t* const msg)
{
  if (GPS_.enabled)
  {
    gps_velEcef_.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1e3);
    gps_velEcef_.vector.x = msg->vel[0];
    gps_velEcef_.vector.y = msg->vel[1];
    gps_velEcef_.vector.z = msg->vel[2];
    publishGPS();
  }
}

void InertialSenseROS::publishGPS()
{
  if (gps_velEcef_.header.stamp == gps_msg_.header.stamp)
  {
    gps_msg_.vel_ecef.x = gps_velEcef_.vector.x;
    gps_msg_.vel_ecef.y = gps_velEcef_.vector.y;
    gps_msg_.vel_ecef.z = gps_velEcef_.vector.z;
    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<inertial_sense_ros2_v2::msg::GPS>>(GPS_.pub);
    if (pub) pub->publish(gps_msg_);
  }
}

void InertialSenseROS::GPS_info_callback(const gps_sat_t* const msg)
{
  gps_info_msg_.header.stamp    = ros_time_from_tow(msg->timeOfWeekMs / 1e3);
  gps_info_msg_.header.frame_id = frame_id_;
  gps_info_msg_.num_sats        = msg->numSats;
  for (int i = 0; i < 50; ++i)
  {
    gps_info_msg_.satellite_info[i].sat_id = msg->sat[i].svId;
    gps_info_msg_.satellite_info[i].cno   = msg->sat[i].cno;
  }
  auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<inertial_sense_ros2_v2::msg::GPSInfo>>(GPS_info_.pub);
  if (pub) pub->publish(gps_info_msg_);
}

// ============================================================
//  GPS raw / observations
// ============================================================
void InertialSenseROS::GPS_raw_callback(const gps_raw_t* const msg)
{
  switch (msg->dataType)
  {
    case raw_data_type_observation:
      GPS_obs_callback(reinterpret_cast<const obsd_t*>(&msg->data.obs), msg->obsCount);
      break;
    case raw_data_type_ephemeris:
      GPS_eph_callback(reinterpret_cast<const eph_t*>(&msg->data.eph));
      break;
    case raw_data_type_glonass_ephemeris:
      GPS_geph_callback(reinterpret_cast<const geph_t*>(&msg->data.gloEph));
      break;
    default:
      break;
  }
}

void InertialSenseROS::GPS_obs_callback(const obsd_t* const msg, int nObs)
{
  if (!obs_Vec_.obs.empty() &&
      (msg[0].time.time != obs_Vec_.obs[0].time.time ||
       msg[0].time.sec  != obs_Vec_.obs[0].time.sec))
  {
    GPS_obs_bundle_timer_callback();
  }

  for (int i = 0; i < nObs; ++i)
  {
    inertial_sense_ros2_v2::msg::GNSSObservation obs;
    if (!obs_Vec_.obs.empty())
      obs.header.stamp = ros_time_from_gtime(obs_Vec_.obs[0].time.time, obs_Vec_.obs[0].time.sec);
    obs.time.time  = msg[i].time.time;
    obs.time.sec   = msg[i].time.sec;
    obs.sat        = msg[i].sat;
    obs.rcv        = msg[i].rcv;
    obs.snr        = msg[i].SNR[0];
    obs.lli        = msg[i].LLI[0];
    obs.code       = msg[i].code[0];
    obs.qual_l     = msg[i].qualL[0];
    obs.qual_p     = msg[i].qualP[0];
    obs.l          = msg[i].L[0];
    obs.p          = msg[i].P[0];
    obs.d          = msg[i].D[0];
    obs_Vec_.obs.push_back(obs);
    last_obs_time_ = this->now();
  }
}

void InertialSenseROS::GPS_obs_bundle_timer_callback()
{
  if (obs_Vec_.obs.empty()) return;

  if ((this->now() - last_obs_time_).seconds() > 1e-2)
  {
    obs_Vec_.header.stamp = ros_time_from_gtime(obs_Vec_.obs[0].time.time,
                                                 obs_Vec_.obs[0].time.sec);
    obs_Vec_.time = obs_Vec_.obs[0].time;
    auto pub = std::dynamic_pointer_cast<
      rclcpp::Publisher<inertial_sense_ros2_v2::msg::GNSSObsVec>>(GPS_obs_.pub);
    if (pub) pub->publish(obs_Vec_);
    obs_Vec_.obs.clear();
  }
}

void InertialSenseROS::GPS_eph_callback(const eph_t* const msg)
{
  inertial_sense_ros2_v2::msg::GNSSEphemeris eph;
  eph.sat       = msg->sat;
  eph.iode      = msg->iode;
  eph.iodc      = msg->iodc;
  eph.sva       = msg->sva;
  eph.svh       = msg->svh;
  eph.week      = msg->week;
  eph.code      = msg->code;
  eph.flag      = msg->flag;
  eph.toe.time  = msg->toe.time;
  eph.toc.time  = msg->toc.time;
  eph.ttr.time  = msg->ttr.time;
  eph.toe.sec   = msg->toe.sec;
  eph.toc.sec   = msg->toc.sec;
  eph.ttr.sec   = msg->ttr.sec;
  eph.a         = msg->A;
  eph.e         = msg->e;
  eph.i0        = msg->i0;
  eph.omg0      = msg->OMG0;
  eph.omg       = msg->omg;
  eph.m0        = msg->M0;
  eph.deln      = msg->deln;
  eph.omgd      = msg->OMGd;
  eph.idot      = msg->idot;
  eph.crc       = msg->crc;
  eph.crs       = msg->crs;
  eph.cuc       = msg->cuc;
  eph.cus       = msg->cus;
  eph.cic       = msg->cic;
  eph.cis       = msg->cis;
  eph.toes      = msg->toes;
  eph.fit       = msg->fit;
  eph.f0        = msg->f0;
  eph.f1        = msg->f1;
  eph.f2        = msg->f2;
  for (int i = 0; i < 4; ++i) eph.tgd[i] = msg->tgd[i];
  eph.adot      = msg->Adot;
  eph.ndot      = msg->ndot;

  auto pub = std::dynamic_pointer_cast<
    rclcpp::Publisher<inertial_sense_ros2_v2::msg::GNSSEphemeris>>(GPS_eph_.pub);
  if (pub) pub->publish(eph);
}

void InertialSenseROS::GPS_geph_callback(const geph_t* const msg)
{
  inertial_sense_ros2_v2::msg::GlonassEphemeris geph;
  geph.sat      = msg->sat;
  geph.iode     = msg->iode;
  geph.frq      = msg->frq;
  geph.svh      = msg->svh;
  geph.sva      = msg->sva;
  geph.age      = msg->age;
  geph.toe.time = msg->toe.time;
  geph.tof.time = msg->tof.time;
  geph.toe.sec  = msg->toe.sec;
  geph.tof.sec  = msg->tof.sec;
  for (int i = 0; i < 3; ++i)
  {
    geph.pos[i] = msg->pos[i];
    geph.vel[i] = msg->vel[i];
    geph.acc[i] = msg->acc[i];
  }
  geph.taun  = msg->taun;
  geph.gamn  = msg->gamn;
  geph.dtaun = msg->dtaun;

  auto pub = std::dynamic_pointer_cast<
    rclcpp::Publisher<inertial_sense_ros2_v2::msg::GlonassEphemeris>>(GPS_eph_.pub2);
  if (pub) pub->publish(geph);
}

// ============================================================
//  Magnetometer
// ============================================================
void InertialSenseROS::mag_callback(const magnetometer_t* const msg)
{
  sensor_msgs::msg::MagneticField mag_msg;
  mag_msg.header.stamp    = ros_time_from_start_time(msg->time);
  mag_msg.header.frame_id = frame_id_;
  mag_msg.magnetic_field.x = msg->mag[0];
  mag_msg.magnetic_field.y = msg->mag[1];
  mag_msg.magnetic_field.z = msg->mag[2];

  auto pub = std::dynamic_pointer_cast<
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>>(mag_.pub);
  if (pub) pub->publish(mag_msg);
}

// ============================================================
//  Barometer
// ============================================================
void InertialSenseROS::baro_callback(const barometer_t* const msg)
{
  sensor_msgs::msg::FluidPressure baro_msg;
  baro_msg.header.stamp    = ros_time_from_start_time(msg->time);
  baro_msg.header.frame_id = frame_id_;
  baro_msg.fluid_pressure  = msg->bar;

  auto pub = std::dynamic_pointer_cast<
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>>(baro_.pub);
  if (pub) pub->publish(baro_msg);
}

// ============================================================
//  Pre-integrated IMU  — SDK 1.12: pimu_t (was preintegrated_imu_t)
// ============================================================
void InertialSenseROS::preint_IMU_callback(const pimu_t* const msg)
{
  inertial_sense_ros2_v2::msg::PreIntIMU preintIMU_msg;
  preintIMU_msg.header.stamp    = ros_time_from_start_time(msg->time);
  preintIMU_msg.header.frame_id = frame_id_;
  // SDK 1.12: fields are theta[3] and vel[3] (was theta1/vel1)
  preintIMU_msg.dtheta.x = msg->theta[0];
  preintIMU_msg.dtheta.y = msg->theta[1];
  preintIMU_msg.dtheta.z = msg->theta[2];
  preintIMU_msg.dvel.x   = msg->vel[0];
  preintIMU_msg.dvel.y   = msg->vel[1];
  preintIMU_msg.dvel.z   = msg->vel[2];
  preintIMU_msg.dt       = msg->dt;

  auto pub = std::dynamic_pointer_cast<
    rclcpp::Publisher<inertial_sense_ros2_v2::msg::PreIntIMU>>(dt_vel_.pub);
  if (pub) pub->publish(preintIMU_msg);
}

// ============================================================
//  Strobe
// ============================================================
void InertialSenseROS::strobe_in_time_callback(const strobe_in_time_t* const msg)
{
  if (!strobe_pub_)
    strobe_pub_ = this->create_publisher<std_msgs::msg::Header>("strobe_time", 1);

  std_msgs::msg::Header strobe_msg;
  strobe_msg.stamp = ros_time_from_week_and_tow(msg->week, msg->timeOfWeekMs * 1e-3);
  strobe_pub_->publish(strobe_msg);
}

// ============================================================
//  RTK callbacks
// ============================================================
void InertialSenseROS::RTK_Misc_callback(const gps_rtk_misc_t* const msg)
{
  if (!RTK_.enabled) return;

  inertial_sense_ros2_v2::msg::RTKInfo rtk_info;
  rtk_info.header.stamp = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1000.0);
  rtk_info.base_ant_count = msg->baseAntennaCount;
  rtk_info.base_eph = msg->baseBeidouEphemerisCount  + msg->baseGalileoEphemerisCount
                    + msg->baseGlonassEphemerisCount  + msg->baseGpsEphemerisCount;
  rtk_info.base_obs = msg->baseBeidouObservationCount + msg->baseGalileoObservationCount
                    + msg->baseGlonassObservationCount + msg->baseGpsObservationCount;
  rtk_info.base_lla[0] = static_cast<float>(msg->baseLla[0]);
  rtk_info.base_lla[1] = static_cast<float>(msg->baseLla[1]);
  rtk_info.base_lla[2] = static_cast<float>(msg->baseLla[2]);
  rtk_info.rover_eph = msg->roverBeidouEphemerisCount  + msg->roverGalileoEphemerisCount
                     + msg->roverGlonassEphemerisCount  + msg->roverGpsEphemerisCount;
  rtk_info.rover_obs = msg->roverBeidouObservationCount + msg->roverGalileoObservationCount
                     + msg->roverGlonassObservationCount + msg->roverGpsObservationCount;
  rtk_info.cycle_slip_count = msg->cycleSlipCount;

  auto pub = std::dynamic_pointer_cast<
    rclcpp::Publisher<inertial_sense_ros2_v2::msg::RTKInfo>>(RTK_.pub);
  if (pub) pub->publish(rtk_info);
}

void InertialSenseROS::RTK_Rel_callback(const gps_rtk_rel_t* const msg)
{
  if (!RTK_.enabled) return;

  inertial_sense_ros2_v2::msg::RTKRel rtk_rel;
  rtk_rel.header.stamp      = ros_time_from_week_and_tow(GPS_week_, msg->timeOfWeekMs / 1000.0);
  rtk_rel.differential_age  = msg->differentialAge;
  rtk_rel.ar_ratio          = msg->arRatio;
  // SDK 1.12: renamed from vectorToBase/distanceToBase/headingToBase
  rtk_rel.vector_to_base.x  = msg->baseToRoverVector[0];
  rtk_rel.vector_to_base.y  = msg->baseToRoverVector[1];
  rtk_rel.vector_to_base.z  = msg->baseToRoverVector[2];
  rtk_rel.distance_to_base  = msg->baseToRoverDistance;
  rtk_rel.heading_to_base   = msg->baseToRoverHeading;

  auto pub = std::dynamic_pointer_cast<
    rclcpp::Publisher<inertial_sense_ros2_v2::msg::RTKRel>>(RTK_.pub2);
  if (pub) pub->publish(rtk_rel);
}

// ============================================================
//  Wheel encoder
// ============================================================
void InertialSenseROS::wheel_enc_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() < 2 || msg->velocity.size() < 2) return;

  wheel_encoder_t wheel_enc_msg;
  wheel_enc_msg.timeOfWeek = tow_from_ros_time(
    rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec));
  wheel_enc_msg.status  = 0;
  wheel_enc_msg.theta_l = static_cast<float>(msg->position[0]);
  wheel_enc_msg.theta_r = static_cast<float>(msg->position[1]);
  wheel_enc_msg.omega_l = static_cast<float>(msg->velocity[0]);
  wheel_enc_msg.omega_r = static_cast<float>(msg->velocity[1]);
  IS_.SendData(DID_WHEEL_ENCODER,
               reinterpret_cast<uint8_t*>(&wheel_enc_msg), sizeof(wheel_encoder_t), 0);
}

// ============================================================
//  Services
// ============================================================
bool InertialSenseROS::set_current_position_as_refLLA(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(&lla_),
               sizeof(lla_), offsetof(nvm_flash_cfg_t, refLla));
  res->success = true;
  return true;
}

bool InertialSenseROS::set_refLLA_to_value(
  const inertial_sense_ros2_v2::srv::RefLLAUpdate::Request::SharedPtr req,
  inertial_sense_ros2_v2::srv::RefLLAUpdate::Response::SharedPtr res)
{
  double lla[3] = { req->lla[0], req->lla[1], req->lla[2] };
  IS_.SendData(DID_FLASH_CONFIG, reinterpret_cast<uint8_t*>(lla),
               sizeof(lla), offsetof(nvm_flash_cfg_t, refLla));
  RCLCPP_INFO(this->get_logger(), "refLLA updated");
  res->success = true;
  return true;
}

bool InertialSenseROS::perform_mag_cal_srv_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  // SDK 1.12: mag_cal_t::enMagRecal renamed to mag_cal_t::state. 2=single-axis, 1=multi-axis
  uint32_t cmd = 2;  // single-axis recalibration
  IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t*>(&cmd),
               sizeof(uint32_t), offsetof(mag_cal_t, state));
  res->success = true;
  return true;
}

bool InertialSenseROS::perform_multi_mag_cal_srv_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  uint32_t cmd = 1;  // multi-axis recalibration
  IS_.SendData(DID_MAG_CAL, reinterpret_cast<uint8_t*>(&cmd),
               sizeof(uint32_t), offsetof(mag_cal_t, state));
  res->success = true;
  return true;
}

bool InertialSenseROS::update_firmware_srv_callback(
  const inertial_sense_ros2_v2::srv::FirmwareUpdate::Request::SharedPtr /*req*/,
  inertial_sense_ros2_v2::srv::FirmwareUpdate::Response::SharedPtr res)
{
  // SDK 1.12.0 changed BootloadFile signature significantly.
  // Firmware updates should be performed using the InertialSense CLTool directly.
  RCLCPP_WARN(this->get_logger(),
    "Firmware update via ROS service not supported in SDK 1.12.0. "
    "Use the InertialSense CLTool (cltool -c /dev/ttyACM0 -ufirmware.hex).");
  res->success = false;
  res->message = "Use InertialSense CLTool for firmware updates with SDK 1.12.0";
  return false;
}

// ============================================================
//  Time utilities
// ============================================================
rclcpp::Time InertialSenseROS::ros_time_from_week_and_tow(uint32_t week, double timeOfWeek)
{
  if (GPS_towOffset_)
  {
    uint64_t sec  = UNIX_TO_GPS_OFFSET + static_cast<uint64_t>(floor(timeOfWeek)) +
                    static_cast<uint64_t>(week) * 7 * 24 * 3600;
    uint32_t nsec = static_cast<uint32_t>((timeOfWeek - floor(timeOfWeek)) * 1e9);
    return rclcpp::Time(static_cast<int32_t>(sec), nsec);
  }
  else
  {
    if (!got_first_message_)
    {
      got_first_message_ = true;
      INS_local_offset_  = this->now().seconds() - timeOfWeek;
    }
    else
    {
      double y_offset   = this->now().seconds() - timeOfWeek;
      INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
    }
    double t = INS_local_offset_ + timeOfWeek;
    return rclcpp::Time(static_cast<int32_t>(t),
                        static_cast<uint32_t>((t - floor(t)) * 1e9));
  }
}

rclcpp::Time InertialSenseROS::ros_time_from_start_time(double time)
{
  if (GPS_towOffset_ > 0.001)
  {
    double full = time + GPS_towOffset_;
    uint64_t sec  = UNIX_TO_GPS_OFFSET + static_cast<uint64_t>(floor(full)) +
                    GPS_week_ * 7 * 24 * 3600;
    uint32_t nsec = static_cast<uint32_t>((full - floor(full)) * 1e9);
    return rclcpp::Time(static_cast<int32_t>(sec), nsec);
  }
  else
  {
    if (!got_first_message_)
    {
      got_first_message_ = true;
      INS_local_offset_  = this->now().seconds() - time;
    }
    else
    {
      double y_offset   = this->now().seconds() - time;
      INS_local_offset_ = 0.005 * y_offset + 0.995 * INS_local_offset_;
    }
    double t = INS_local_offset_ + time;
    return rclcpp::Time(static_cast<int32_t>(t),
                        static_cast<uint32_t>((t - floor(t)) * 1e9));
  }
}

rclcpp::Time InertialSenseROS::ros_time_from_tow(double tow)
{
  return ros_time_from_week_and_tow(static_cast<uint32_t>(GPS_week_), tow);
}

double InertialSenseROS::tow_from_ros_time(const rclcpp::Time& rt)
{
  double sec = rt.seconds();
  return (sec - UNIX_TO_GPS_OFFSET - static_cast<double>(GPS_week_) * 604800.0);
}

rclcpp::Time InertialSenseROS::ros_time_from_gtime(uint64_t sec, double subsec)
{
  return rclcpp::Time(static_cast<int32_t>(sec - LEAP_SECONDS),
                      static_cast<uint32_t>(subsec * 1e9));
}

// ============================================================
//  main
// ============================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InertialSenseROS>();

  rclcpp::Rate rate(1000);  // 1kHz spin — SDK update() does the actual work
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    node->update();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
