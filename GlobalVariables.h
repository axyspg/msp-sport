#define PIDITEMS 10

/********************       For Sensors presence      *********************/
#define ACCELEROMETER  1//0b00000001
#define BAROMETER      2//0b00000010
#define MAGNETOMETER   4//0b00000100
#define GPSSENSOR      8//0b00001000
#define SONAR         16//0b00010000

//General use variables
struct {
//  uint8_t tenthSec;
//  uint8_t halfSec;
//  uint8_t Blink2hz;                          // This is turing on and off at 2hz
//  uint8_t Blink10hz;                         // This is turing on and off at 10hz
//  uint16_t lastCallSign;                     // Callsign_timer
//  uint8_t rssiTimer;
//  uint8_t accCalibrationTimer;
//  uint8_t magCalibrationTimer;
//  uint32_t fwAltitudeTimer;
  uint32_t seconds;
  uint8_t MSP_active;
  uint8_t GPS_active;
}
timer;

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

int32_t  MwAltitude=0;                         // This hold barometric value

int MwAngle[2]={0,0};           // Those will hold Accelerator Angle

uint16_t MwSensorPresent=0;
uint32_t MwSensorActive=0;
//uint8_t MwArmedFlags = 0;
uint8_t MwStateFlags = 0;
//uint8_t MwVersion=0;
uint8_t MwVBat=0;
int16_t MwVario=0;
uint8_t MwProfile=0;
//uint8_t armed=0;
//uint8_t previousarmedstatus=0;  // for statistics after disarming
//uint16_t armedangle=0;           // for capturing direction at arming
uint32_t GPS_distanceToHome=0;
uint8_t GPS_fix=0;
//uint8_t GPS_frame_timer=0;
int32_t GPS_latitude;
int32_t GPS_longitude;
int16_t GPS_altitude;
//int16_t GPS_home_altitude;
//int16_t previousfwaltitude=0;
//int16_t interimfwaltitude=0;
uint16_t GPS_speed;
int16_t  GPS_ground_course;
//uint16_t old_GPS_speed;
int16_t GPS_directionToHome=0;
uint8_t GPS_numSat=0;
//uint8_t GPS_waypoint_step=0;
uint16_t I2CError=0;
uint16_t cycleTime=0;
uint16_t pMeterSum=0;
uint16_t MwRssi=0;
uint32_t GPS_time = 0;        //local time of coord calc - haydent

// For Heading
static int16_t MwHeading=0;

// For Amperage
uint16_t MWAmperage=0;

// ---------------------------------------------------------------------------------------
// Defines imported from Multiwii Serial Protocol MultiWii_shared svn r1337
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
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
//#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
//#define MSP_NAV_STATUS           121   //out message	     Returns navigation status

//#define MSP_CELLS                130   //out message         FrSky SPort Telemtry

/*#define MSP_SET_RAW_RC           200   //in message          8 rc chan
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

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4
*/

// Baseflight specific
/*#define MSP_SET_CONFIG           67    //in message          baseflight-specific settings save
#define MSP_CONFIG               66    //out message         baseflight-specific settings that aren't covered elsewhere
#define SETCONGFIG 25                  //for BASEFLIGHT20150627
*/

// End of imported defines from Multiwii Serial Protocol MultiWii_shared svn r1337
// ---------------------------------------------------------------------------------------
