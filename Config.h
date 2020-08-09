/********************   Protocol settings   *********************/
#define MSPV2				// Enable MSPv2 support		

/******************** Serial speed settings *********************/
// Choose ONLY ONE option: (match with flight controller and MWOSD baudrate)
#define BAUDRATE 115200
//#define BAUDRATE 57600
//#define BAUDRATE 38400
//#define BAUDRATE 19200
//#define BAUDRATE 9600

/******************** Miscellaneous *****************************/
#define ISP_MODE            // if defined, we will enter in ISP mode at powerup
#define ISP_TIMEOUT   5000  // in milliseconds
#define MINIMOSD_RST  SS    // where is MinimOSD reset pin connected
