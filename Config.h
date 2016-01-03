/********************       WARNING/STATUS settings      *********************/
#define GPSACTIVECHECK 5            // Alerts if no GPS data for more than x secs. Sets GPS sats to zero
#define MSPACTIVECHECK 3            // Alerts if no Flight controller data for more than x secs. 
#define INCANGLE       25           // Alerts if craft's angle is greater or equal to this value

/******************** Serial speed settings *********************/
// Choose ONLY ONE option: (match with CleanFlight and MWOSD baudrate)
#define BAUDRATE 115200
//#define BAUDRATE 57600
//#define BAUDRATE 38400
//#define BAUDRATE 19200
//#define BAUDRATE 9600

/******************** Miscellaneous *****************************/
#define CFTS_ENABLED        // define this if you are using Axys' CleanFlight Telemetry Script                              
                            // More info at http://someonedidthatalready.blogspot.com
                            
#define ISP_MODE            // if defined, we will enter in ISP mode at powerup
#define ISP_TIMEOUT   5000  // in milliseconds
#define MINIMOSD_RST  SS    // where is MinimOSD reset pin connected

