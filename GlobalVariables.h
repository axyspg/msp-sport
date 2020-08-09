#define PIDITEMS 10

/********************       For Sensors presence      *********************/
#define ACCELEROMETER  1//0b00000001
#define BAROMETER      2//0b00000010
#define MAGNETOMETER   4//0b00000100
#define GPSSENSOR      8//0b00001000
#define SONAR         16//0b00010000

// inav ids
#define IDBOXAIR 29
#define IDBOXWP 28
#define IDBOXGPSLAND 126 // random unused to disable

//General use variables
struct __datetime {
  uint32_t unixtime;
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hours;
  uint8_t  minutes;  
  uint8_t  seconds;  
} datetime;

// Mode bits
struct __mode {
  uint8_t armed;
  uint8_t stable;
  uint8_t horizon;
  uint32_t baro;
  uint32_t mag;
  uint32_t camstab;
  uint32_t gpshome;
  uint32_t gpshold;
  uint32_t passthru;
  uint32_t failsafe;
  uint32_t air;
  uint32_t acroplus;
  uint32_t osd_switch;
  uint32_t llights;
  uint32_t gpsmission;
  uint32_t cruise;
//#ifdef EXTENDEDMODESUPPORT
  uint32_t gpsland;
  uint32_t autotune;
  uint32_t autotrim;
  uint32_t launch;
  uint32_t flaperon;
//#endif
}mode;

//struct {
//  uint8_t tenthSec;
//  uint8_t halfSec;
//  uint8_t Blink2hz;                          // This is turing on and off at 2hz
//  uint8_t Blink10hz;                         // This is turing on and off at 10hz
//  uint16_t lastCallSign;                     // Callsign_timer
//  uint8_t rssiTimer;
//  uint8_t accCalibrationTimer;
//  uint8_t magCalibrationTimer;
//  uint32_t fwAltitudeTimer;
//  uint32_t seconds;
//  uint8_t MSP_active;
//  uint8_t GPS_active;
//}
//timer;

/*
static uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];

static uint8_t rcRate8,rcExpo8;
static uint8_t rollPitchRate;
static uint8_t rollRate;
static uint8_t PitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint8_t thrMid8;
static uint8_t thrExpo8;
static uint16_t tpa_breakpoint16;
static uint8_t rcYawExpo8;
*/

//int32_t  MwAltitude=0;                         // This hold barometric value

//int gAngle[2] = { 0, 0 };           // Those will hold Accelerator Angle

//uint16_t MwSensorPresent=0;
uint32_t gMwSensorActive = 0;
//uint8_t MwArmedFlags = 0;
//uint8_t MwStateFlags = 0;
//uint8_t MwVersion=0;
//uint8_t MwVBat=0;
//int16_t MwVario=0;
//uint8_t MwProfile=0;
//uint8_t armed=0;
//uint8_t previousarmedstatus=0;  // for statistics after disarming
//uint16_t armedangle=0;           // for capturing direction at arming
//uint8_t GPS_fix=0;
//uint8_t GPS_frame_timer=0;
//int16_t GPS_home_altitude;
//int16_t previousfwaltitude=0;
//int16_t interimfwaltitude=0;
//uint16_t old_GPS_speed;
//uint8_t GPS_numSat=0;
//uint8_t GPS_waypoint_step=0;
//uint16_t I2CError=0;
//uint16_t cycleTime=0;
//uint16_t pMeterSum=0;
uint16_t gRssi = 0;

uint16_t gVBat = 0;
//uint16_t gCell_data[6]={0,0,0,0,0,0};
uint8_t  gNumCells = 0;

// For Angles
//int16_t  gPitch = 0;
//int16_t  gRaw = 0;
int16_t  gHeading = 0;

// For GPS
bool 	   gGPS_fix = false;
uint8_t  gGPS_sats = 0;
bool 	   gGPS_homeFix = false;
int32_t  gGPS_latitude = 0;
int32_t  gGPS_longitude = 0;
int16_t  gGPS_altitude = 0;
uint16_t gGPS_speed = 0;
int16_t  gGPS_groundCourse;
uint32_t gGPS_distanceToHome = 0;
int16_t  gGPS_directionToHome = 0;
uint16_t gGPS_hdop = 0;
//uint32_t gGPS_time = 0;        		//local time of coord calc - haydent

int16_t gAIR_speed = 0;

// For Amperage
//uint16_t MWAmperage=0;
uint32_t gDebug1 = 0, gDebug2 = 0, gDebug3 = 0; // DEBUG

// ---------------------------------------------------------------------------------------
// Defines imported from Multiwii Serial Protocol MultiWii_shared svn r1337
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
//#define MSP_FC_VERSION             3   //out message         FC firmware version
//#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
//#define MSP_RAW_IMU              102   //out message         9 DOF
//#define MSP_SERVO                103   //out message         8 servos
//#define MSP_MOTOR                104   //out message         8 motors
//#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
//#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
//#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
//#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
//#define MSP_MISC                 114   //out message         powermeter trig
//#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
//#define MSP_BOXNAMES             116   //out message         the aux switch names
//#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
//#define MSP_SERVO_CONF           120    //out message         Servo settings
//#define MSP_NAV_STATUS           121   //out message	     Returns navigation status

#define MSP_CELLS                130   //out message         FrSky SPort Telemtry

//#define MSP_DISPLAYPORT          182

/*
#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212    //in message          Servo settings

#define MSP_BIND                 240   //in message          no param

#define MSP_ALARMS               242   //in message          poll for alert text

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4
*/

// iNAV specific
//#define MSP_GPSSTATISTICS        166    //out message         get GPS debugging data
#define MSP_SENSOR_STATUS        151    //out message         Gets the sensor HW status. Bit 0 = overall health
#define MSP_RTC                  246    //out message         Gets the RTC clock (returns: secs(i32) millis(u16) - (0,0) if time is not known)

// iNAV MSPV2 specific
#define MSP2_INAV_ANALOG         0x2002    //in messages      Returns iNAV extended analog  
#define MSP2_INAV_AIR_SPEED      0x2009    //in message       Returns airspeed

// End of imported defines from Multiwii Serial Protocol MultiWii_shared svn r1337
// ---------------------------------------------------------------------------------------
