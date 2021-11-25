#ifndef _HAL_H_
#define _HAL_H_

#define _ver   1.01
// Hardware definitions
//Buttons
#define	BT1_pin                             0   //Keyboard
#define	BT2_pin                             12  //Keyboard
#define	BT3_pin                             13  //Keyboard
#define	BT4_pin                             14  //Keyboard //2
#define	BT5_pin                             15  //Joystick

// HW IO Expander
#define exp_io_MTR_AZM_EN_pin               0   //Stepper Motor
#define exp_io_MTR_AZM_DIR_pin              1   //Stepper Motor
#define exp_io_MTR_AZM_MODE2_pin            2   //Stepper Motor
#define exp_io_MTR_AZM_SLP_FLT_pin          3   //Stepper Motor
#define exp_io_MTR_ALT_EN_pin               4   //Stepper Motor
#define exp_io_MTR_ALT_DIR_pin              5   //Stepper Motor
#define exp_io_MTR_ALT_MODE2_pin            6   //Stepper Motor
#define exp_io_MTR_ALT_SLP_FLT_pin          7   //Stepper Motor
#define exp_io_CHARG_PG_pin                 8   //Charger Li Ion
#define exp_io_CHARG_ST1_pin                9   //Charger Li Ion
#define exp_io_CHARG_ST2_pin                10  //Charger Li Ion
#define exp_io_CHARG_CE_pin                 11  //Charger Li Ion
#define exp_io_CHARG_PROG2_pin              12  //Charger Li Ion
#define exp_io_CHARG_TE_pin                 13  //Charger Li Ion
#define exp_io_DSP_RST_pin                  14  //Display
#define exp_io_DSP_DC_pin                   15  //Display

// AD Joystick
#define ADC_JOY_X_pin                       36 //VDET_2 //4  //PINO REFERENTE A LIGAÇÃO DO EIXO y
#define ADC_JOY_Y_pin                       39 //VDET_1 //5  //PINO REFERENTE A LIGAÇÃO DO EIXO X

// AD Voltage
#define ADC_VUSB_pin                        34
#define ADC_VBAT_pin                        35

// Serial
#define RXD0_pin                            3
#define TXD0_pin                            1

#define GPS_RX_pin                          16
#define GPS_TX_pin                          17

#define DBG_RX_pin                          9
#define DBG_TX_pin                          10

// Buzzer
#define BUZZER_pin                          8

//Stepper Motor
#define MTR_AZM_STEP_pin                    4
#define MTR_ALT_STEP_pin                    5

#define SOURCE_3V3_EN_pin                   19
#define GPS_TPULSE_pin                      23
#define RGB_R_pin                           25
#define RGB_G_pin                           26
#define RGB_B_pin                           27

#define SOURCE_DC_ERR_pin                   32
#define SOURCE_15V_EN_pin                   33

// Rename Serials
#define SerialStellarium                    Serial
#define SerialDebug                         Serial1
#define SerialGPS                           Serial2

// Config Motor
#define MOTOR_AZM_RPM                       40
#define MOTOR_ALT_RPM                       40
#define MOTOR_ACCEL                         20
#define MOTOR_DECEL                         10
#define MOTOR_STEPS                         2000    //1,8°/stepp * 10 
#define MICROSTEPS                          8

// Config Display
#define SCREEN_WIDTH                        128
#define SCREEN_HEIGHT                       64
#define SCREEN_ADDRESS                      0x3C

// Config ADC
#define VREF                                3.78
#define RESOLUTION                          4095.0
#define R1                                  100000.0
#define R2                                  100000.0
#define BATTERY_MEAS_MIN_LEVEL              3100
#define BATTERY_MEAS_MAX_LEVEL              4200
#define VOLTAGE_TO_PERCENT_DELTA            10

// Define to Menu
#define OFFSET_MN_ICON_1                    40
#define OFFSET_MN_ICON_2                    52
#define OFFSET_MN_ICON_3                    65
#define OFFSET_MN_ICON_4                    78
#define OFFSET_MN_ICON_5                    91
#define OFFSET_MN_ICON_6                    104
#define OFFSET_MN_ICON_7                    116
#define OFFSET_MN_MODE                      0
#define OFFSET_MN_BT_1                      32
#define OFFSET_MN_BT_2                      56
#define OFFSET_MN_BT_3                      80
#define OFFSET_MN_BT_4                      104


// Converting measured battery voltage[mV] to State of Charge[%].
static const uint8_t battery_voltage_to_percent[] = {
  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,
  2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,
  4,  5,  5,  5,  6,  6,  7,  7,  8,  8,  9,  9, 10, 11, 12, 13, 13, 14, 15, 16,
  18, 19, 22, 25, 28, 32, 36, 40, 44, 47, 51, 53, 56, 58, 60, 62, 64, 66, 67, 69,
  71, 72, 74, 76, 77, 79, 81, 82, 84, 85, 85, 86, 86, 86, 87, 88, 88, 89, 90, 91,
  91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 100
};

//IIR Filter
//const double b_coefficients[] = { b_0, b_1, b_2, ... , b_P };
//const double a_coefficients[] = { a_0, a_1, a_2, ... , a_Q };
// 35 Hz Butterworth low-pass
const double b_lp[] = { 0.00113722762905776, 0.00568613814528881, 0.0113722762905776,  0.0113722762905776,  0.00568613814528881, 0.00113722762905776 };
const double a_lp[] = { 1, -3.03124451613593, 3.92924380774061,  -2.65660499035499, 0.928185738776705, -0.133188755896548 };
static float COEF_A[4] = {
  1,
  -1.1619,
  0.69594,
  -0.13776
};

static float COEF_B[4] = {
  0.049533,
  0.1486,
  0.1486,
  0.049533
};

// Config Buttons
#define MAX_BUTTONS                         5
#define DEBOUNCE_TIME                       40
#define SHORT_TIME                          4000
#define LONG_TIME                           4000

struct Button {
    const uint8_t PIN;
    bool state;
    bool old_state;
    bool pressed_short;
    bool pressed_long;
};

Button button[MAX_BUTTONS] =  {BT1_pin, true, true, false, false,
                               BT2_pin, true, true, false, false,
                               BT3_pin, true, true, false, false,
                               BT4_pin, true, true, false, false,
                               BT5_pin, true, true, false, false};

// Data Struct
struct DMS_t{
  int D;
  int M;
  int S;
  int SS;
};

struct HMS_t{
  int H;
  int M;
  int S;
  int SS;
};

struct Coord_DMS_t{
  double DD;  //decimal degree
  DMS_t DMS;
};

struct Coord_HMS_t{
  double DD;  //decimal degree
  double DT;  //decimal time
  HMS_t HMS;
};

struct data_t
{
    struct
    {
        uint16_t Joystick_X;
        uint16_t Joystick_Y;
        uint16_t VBat;
        uint16_t VUSB;
    }ADC;
  struct 
  {
    uint8_t Joystick;
    struct
    {
      uint8_t Mode; //0 - Disable / 1 - Enable Stellarium / 2 - Go Target Fast
      uint8_t Control;
      uint8_t SerialBle;  //0 - Serial / 1 - BLE
    }Stellarium;
  }Control;

  struct 
  {
    uint8_t Menu;
    uint8_t SubMenu;
  }Display;

  struct
  {
    uint32_t iTOW; // GPS time of week of the navigation epoch: ms
    uint16_t year; // Year (UTC)
    uint8_t month; // Month, range 1..12 (UTC)
    uint8_t day; // Day of month, range 1..31 (UTC)
    uint8_t hour; // Hour of day, range 0..23 (UTC)
    uint8_t min; // Minute of hour, range 0..59 (UTC)
    uint8_t sec; // Seconds of minute, range 0..60 (UTC)
    union
    {
      uint8_t all;
      struct
      {
        uint8_t validDate : 1; // 1 = valid UTC Date
        uint8_t validTime : 1; // 1 = valid UTC time of day
        uint8_t fullyResolved : 1; // 1 = UTC time of day has been fully resolved (no seconds uncertainty).
        uint8_t validMag : 1; // 1 = valid magnetic declination
      } bits;
    } valid;
    uint32_t tAcc; // Time accuracy estimate (UTC): ns
    int32_t nano; // Fraction of second, range -1e9 .. 1e9 (UTC): ns
    uint8_t fixType; // GNSSfix Type:
                        // 0: no fix
                        // 1: dead reckoning only
                        // 2: 2D-fix
                        // 3: 3D-fix
                        // 4: GNSS + dead reckoning combined
                        // 5: time only fix
    union
    {
      uint8_t all;
      struct
      {
        uint8_t gnssFixOK : 1; // 1 = valid fix (i.e within DOP & accuracy masks)
        uint8_t gnssOK : 1; // 1 = communication valid
      } bits;
    } flags;
    uint8_t numSV; // Number of satellites used in Nav Solution
    float lon; // Longitude: deg * 1e-7
    float lat; // Latitude: deg * 1e-7
    float height; // Height above ellipsoid: mm
    float hMSL; // Height above mean sea level: mm
  }Gps;
  struct
  {
    struct
    {
      double X;
      double Y;
      double Z;
    }LinearAccel;
    struct
    {
      double X;
      double Y;
      double Z;
    }Accelerometer;
    struct
    {
      double X;
      double Y;
      double Z;
    }Gravity;
    struct
    {
      double X;
      double Y;
      double Z;
    }Euler;
    struct
    {
      double X;
      double Y;
      double Z;
    }EulerIIR;
    struct
    {
      double X;
      double Y;
      double Z;
    }Gyroscope;
    struct
    {
      double X;
      double Y;
      double Z;
    }Magnetometer;
    struct
    {
      double W;
      double X;
      double Y;
      double Z;
    }Quarternion;
    struct
    {
        uint8_t MAG;
        uint8_t ACC;
        uint8_t GYR;
        uint8_t SYS;
    } Calibration;
    int8_t Temperature;
    unsigned long  Freq;
  }Inertial;
  struct
  {
    Coord_DMS_t Azimuth;
    Coord_DMS_t Altitude;
    Coord_HMS_t RightAscension;
    Coord_DMS_t Declination;
    double      deltaAzimuthTarget;
    double      deltaAltitudeTarget;
  }Telescope;
  struct
  {
      Coord_DMS_t Azimuth;
      Coord_DMS_t Altitude;
      Coord_HMS_t RightAscension;
      Coord_DMS_t Declination;
  }Stellarium;
  struct
  {
      double  UTC;        //Universal Time Coordenate (seg)
      double  TT;         //Terrestrial Time is a modern astronomical time standard defined by the International Astronomical Union
      double  JD;         //Julian Day is a continuous count of days and fractions thereof from the beginning of the year -4712
      double  MJD;        //Modified Julian Day, used in orbital elements of artificial satellites, begins at Greewich mean midnight
      double  JD2000;     //Julian Day since 2000
      double  GMST;       //Greenwich mean sidereal time
      double  LST;        //Local mean sidereal time
      //double  HA;         //Local Right Ascension
  }Time;
};





#endif // _HAL_H_