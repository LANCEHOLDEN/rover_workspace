# GPS status flags
uint32 GPS_STATUS_FIX_TYPE_NO_FIX=0
uint32 GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY=256
uint32 GPS_STATUS_FIX_TYPE_2D_FIX=512
uint32 GPS_STATUS_FIX_TYPE_3D_FIX=768
uint32 GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK=1024
uint32 GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX=1280
uint32 GPS_STATUS_FIX_TYPE_RESERVED1=1536
uint32 GPS_STATUS_FIX_TYPE_RESERVED2=1792

uint32 GPS_STATUS_FIX_STATUS_FIX_OK=65536

std_msgs/Header header
int8 num_sat                        # Number of satellites used in solution
uint32 fix_type                     # Fix type, one of STATUS_FIX_TYPE flags
int32 cno                           # mean carrier noise ratio (dBHz)
float64 latitude                    # latitude (degrees)
float64 longitude                   # longitude (degrees)
float64 altitude                    # height above ellipsoid (not MSL) (m)
geometry_msgs/Vector3 pos_ecef      # Position (m) in ECEF
geometry_msgs/Vector3 vel_ecef      # Velocity (m/s) in ECEF
float32 h_msl                       # height above MSL
float32 h_acc                       # horizontal accuracy
float32 v_acc                       # vertical accuracy
float32 s_acc                       # speed accuracy (m/s)
float32 p_dop                       # Position Dilution of Precision (m)
