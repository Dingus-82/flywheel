/*
_____________________________________________________________
Variables for Pi Pico W data logger
Code by: Radhakrishna Vojjala
Date of last modification: 21 Apr 2026
_____________________________________________________________
This file contains most of the variables used for the Pi Pico W flight logger.
If other sensors are added, add their variables to this file for cleaner code formatting.
*/

// Variables for the system

#define SERIAL_BAUD 115200
#define ERR_LED_PIN 20
#define LOOP_LED_PIN 21
#define I2C_1_SDA 2
#define I2C_1_SCL 3

int nowTimeMS = 0;
float nowTimeS = 0;
float nowTimeMin = 0;
int prevTime = 0;
int flightTimeMS = 0;
String flightTimeHHMMSS = "";
String HHMMSS = "";
String data = "";
float freq = 0;
int loopTime = 0;
bool error = false;
int Volt = 0;
bool flightStarted = false;
int flightOffset = 0;

Thermistor inTher(A0);
Thermistor outTher(A1); 

bool inStatus = false;
bool outStatus = false;
float inTempF = 0;
float inTempC = 0;
float inTempRK = 0;
float outTempRK = 0;
float outTempF = 0;
float outTempC = 0;

// Variables for SPI SD card reader

#define CS 17    // Chip select pin is GP17
#define MOSI 19  // Master out Slave in (SPI TX) is GP19
#define MISO 16  // Master in Slave out (SPI RX) is GP16
#define SCK 18   // SPI clock signal pin is GP18

File dataLog;
bool SDstatus = true;
bool SDfull = true;
bool filesAvailable = true;
char dataFilename[] = "PICOLO00.csv";  //Make sure dataFileN matches first and second place of the zeros in terms of the arrays index
const byte dataFileN1 = 6;
const byte dataFileN2 = 7;

// Variables for GPS

#define LOCK_LED_PIN 5//22
#define DYN_MODEL DYN_MODEL_AIRBORNE2g

SFE_UBLOX_GNSS sparkFunGNSS;

String GPStype = "M8N";
int gpsBaud = 9600;  // 9600 for M8N, 38400 for M9N
unsigned long gpsTimer = 0;
byte gpsStatus = 0;
int gpsMonth = 999;
int gpsDay = 999;
int gpsYear = 999;
int gpsHour = 999;
int gpsMinute = 999;
int gpsSecond = 999;
int gpsMillisecond = 999;
int gpsTimeOfWeek = 999;
double gpsLat = 999;
double gpsLon = 999;
double gpsAltM = 999;
double gpsAltFt = 999;
double gpsGndSpeed = 999;
double gpsHeading = 999;
double gpsPDOP = 999;
long gpsLatDec = 999;
int gpsLatInt = 999;
long gpsLonDec = 999;
int gpsLonInt = 999;
float altitudeFtGPS = 999;
float latitudeGPS = 999;
float longitudeGPS = 999;
long ecefX = 999, ecefY = 999, ecefZ = 999;
double posAcc = 999;
int SIV = 0;
double latCalc = 999, longCalc = 999, altCalcFt = 999;
double UTMNorthing = 999, UTMEasting = 999;
char UTMZoneLetter = '?';
int UTMZoneNum = 999;
bool pvtStatus = false;
bool ecefStatus = false;
double gpsVertVelFt = 999;
double gpsVertVelM = 999;
unsigned long gpsPrevTime = 0;
double gpsPrevAlt = 0;
double velocityNED[3] = { 999, 999, 999 };
int gpsHorizAcc = 999;
int gpsVertAcc = 999;
String fixTypeGPS = "N/A";  // create an empty string called msgStr with a capacity if 40 chars
const uint8_t currentKeyLengthBytes = 16;
const char currentDynamicKey[] = "7875bc2e9cb9252cef9070aef9ecd091";
const uint16_t currentKeyGPSWeek = 2297;  // Update this when you add new keys
const uint32_t currentKeyGPSToW = 84558;
const uint8_t nextKeyLengthBytes = 16;
const char nextDynamicKey[] = "3fce6f457544996e2e6292b260b52cd7";
const uint16_t nextKeyGPSWeek = 2301;  // Update this when you add new keys
const uint32_t nextKeyGPSToW = 84558;

// Variables for BNO IMU

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
float magnetometer[3] = { 999, 999, 999 };   // {x, y, z}
float accelerometer[3] = { 999, 999, 999 };  // {x, y, z}
float gyroscope[3] = { 999, 999, 999 };      // {x, y, z}
float orientation[3] = { 999, 999, 999 };    // {yaw (rotation about z axis), roll (rotation about y axis), pitch (rotation about x axis)}
float calibration[4] = { 999, 999, 999, 999 };
// Variables for Stemma QT OLED. Other varables may be found in the OLED's header file.

String OLEDstr = "";

// Variables for MS5611

MS5611 ms;
const double R = 287.05; // Gas constant for air
bool msOnline = false;
double refPress = 0;
double MStempC = 999;
double MStempF = 1830.2;  // 999C in F
double MStempRK = 3.14159265359;
double pressPa = 0;
double presskPa = 0;
double pressATM = 0;
double pressPSI = 0;
double absAltM = 0;
double absAltFt = 0;
double relAltM = 0;
double relAltFt = 0;
double vertVelM = 0;
double vertVelFt = 0;
double density = 0;
// Additional variables here

#define ServoStatusLED 22


double BNO055_SAMPLERATE_DELAY_MS = 100;
double targetOrient = 0.0;  // Target heading in degrees
double tolerance = 0.25;  // Tolerance in degrees
int maxServoSpeed = 75;  // Maximum servo speed (0-180)
int minServoSpeed = 13;    // Minimum servo speed (0-180)
int servoOff = 0;     // Neutral position (no rotation)
float orientationError = 0;
float servoCommand = 0;
float pidOutput = 0;
float currentHeading = 0;
float previousError = 0;
unsigned long previousTime = 0;
// PID Constants
float integral = 999;
float proportional = 999;
float prevProp = 999;
float derivative = 999;
//double KP = 0.8;  // Proportional gain
double KP = 1.0;
//double KI = 0.1; // Integral gain  
double KI = 2.0; // Integral gain  
//double KD = 0.05;  // Derivative gain
double KD = 0.15;
int saturationCounter = 0;
int saturationTimer = 0;


//int sdCounter = 0;
