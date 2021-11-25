
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> 
#include <freertos/semphr.h>

#include <esp_deepsleep.h>

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <esp_task_wdt.h>

#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include<TinyGPSPlus.h>
#include <AccelStepper.h>
#include "clsPCA9555.h"

#include "DRV8825.h"
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"

#include "icon.h"
#include "hal.h"

#include "BluetoothSerial.h"
#include <FIRFilter.h>
//#include <IIRFilter.h>

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialDBGSemaphore;
SemaphoreHandle_t xI2CSemaphore;

// define two Tasks
void TaskMain( void *pvParameters );
void TaskMotor( void *pvParameters );
void TaskGPS( void *pvParameters );
void TaskStellarium( void *pvParameters );
void TaskDebug( void *pvParameters );
portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
PCA9555 ioport(0x20);

// Create a new instance of the AccelStepper class:
DRV8825 MTR_AZM (MOTOR_STEPS, exp_io_MTR_AZM_DIR_pin, MTR_AZM_STEP_pin, exp_io_MTR_AZM_EN_pin);
DRV8825 MTR_ALT (MOTOR_STEPS, exp_io_MTR_ALT_DIR_pin, MTR_ALT_STEP_pin, exp_io_MTR_ALT_EN_pin);
//SyncDriver controller(MTR_AZM, MTR_ALT);
MultiDriver controller(MTR_AZM, MTR_ALT);

// Define BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

TinyGPSPlus  GNSS;
BluetoothSerial SerialBT;

//IIRFilter lp(b_lp, a_lp);

data_t Data;

//--------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  xSerialDBGSemaphore = xSemaphoreCreateMutex();  //Create semaphores for SerialDBG
  xSemaphoreGive(xSerialDBGSemaphore);
  xI2CSemaphore = xSemaphoreCreateMutex();        //Create semaphores for I2C
  xSemaphoreGive(xI2CSemaphore);

  gpio_Config();                                  //Configure GPIO's

  ioexpander_Config();                            //Configure IO Expander
  
  display_Config();                               //Configure Display
  display.println("System Initializing:");
  display.display();
  delay(200);

  serial_Config();                                //Configure Serials (DBG, Stellarium)
  buttons_Config();                               //Configure Buttons (debounce, shor and long press)
  led_Config();                                   //Configure LED PWM functionalitites

  //Set RGB 
  ledcWrite(0, 0);  //R
  ledcWrite(1, 2);  //G
  ledcWrite(2, 0);  //B

  gps_Config();                                   //Configure GPS
  motor_Config();                                 //Configure Motor Control
  ble_Config ();                                  //Configure BLE
  sensor_Config();                                //Configure Inertial Sensors
  delay(500);
 
  display.println("System Initialized!");
  display.display();

  delay(1000);

  //Create tasks 
  xTaskCreatePinnedToCore(TaskMain,       "tMain",        2048, NULL, 10, NULL, 0);  
  xTaskCreatePinnedToCore(TaskGPS,        "tGPS",         2048, NULL, 8, NULL, 1);
  xTaskCreatePinnedToCore(TaskStellarium, "tStellarium",  2048, NULL, 9, NULL, 1);
  xTaskCreatePinnedToCore(TaskMotor,      "tMotor",       2048, NULL, 1, NULL, 1);
  //xTaskCreatePinnedToCore(TaskDebug,      "tDebug",       2048, NULL, 4, NULL, 0);
  
}

void loop() 
{
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------

void TaskGPS( void *pvParameters __attribute__((unused)) ){
  static uint32_t lastComm;
  lastComm = xTaskGetTickCount();
  for(;;){
    esp_task_wdt_init(30, false);
    vTaskDelay(1/portTICK_PERIOD_MS);
    while (SerialGPS.available() > 0)
    if (GNSS.encode(SerialGPS.read()))
    {
      lastComm = xTaskGetTickCount();
      taskENTER_CRITICAL(&myMutex);
      Data.Gps.flags.bits.gnssOK = true;
      if (GNSS.date.isValid())
        Data.Gps.valid.bits.validDate = true;
      else
        Data.Gps.valid.bits.validDate = false;

      if (GNSS.time.isValid() > 0)
        Data.Gps.valid.bits.validTime = true;
      else
        Data.Gps.valid.bits.validTime = false;
        if(GNSS.location.isValid())
          Data.Gps.flags.bits.gnssFixOK = true;
        else
          Data.Gps.flags.bits.gnssFixOK = false;    

      Data.Gps.year = GNSS.date.year();
      Data.Gps.month = GNSS.date.month();
      Data.Gps.day = GNSS.date.day();
      Data.Gps.hour = GNSS.time.hour();
      Data.Gps.min = GNSS.time.minute();
      Data.Gps.sec = GNSS.time.second();
      Data.Gps.iTOW = GNSS.time.centisecond();

      if(Data.Gps.flags.bits.gnssFixOK)
      {
        Data.Gps.lat = GNSS.location.lat();
        Data.Gps.lon = GNSS.location.lng();
      }
      else if(Data.Gps.lat == 0 && Data.Gps.lon == 0)
      {
        Data.Gps.lat = -23.5718969;
        Data.Gps.lon = -46.5856458;
      }
      taskEXIT_CRITICAL(&myMutex);
    }
    if (xTaskGetTickCount() - lastComm > 2000 && GNSS.charsProcessed() < 10)
    {
      Data.Gps.flags.bits.gnssOK = false;
    }
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
void TaskStellarium( void *pvParameters __attribute__((unused)) ){
  int i = 0;
  #define MAX_DATA 100
  char data[MAX_DATA];
  struct timeval now;
  
  for(;;){
    esp_task_wdt_init(30, false);
    vTaskDelay(1/portTICK_PERIOD_MS);
    if (Serial.available())
    {
      i = 0;
      data[i++] = Serial.read();

      if (data[0] == '#')
        i = 0;
      vTaskDelay(10/portTICK_PERIOD_MS);

      if(data[0] == 0x06)
        Serial.write('A');

      while ((data[i++] = Serial.read()) != '#' && i < MAX_DATA) {
        vTaskDelay(5/portTICK_PERIOD_MS);
      }
      data[i] = '\0';

      //get time now
      gettimeofday(&now, NULL);

      if(strcmp(data, ":GR#") == 0)
        Serial.printf("%02d:%02d:%02d#", Data.Telescope.RightAscension.HMS.H, Data.Telescope.RightAscension.HMS.M, Data.Telescope.RightAscension.HMS.S, Data.Telescope.RightAscension.HMS.SS);
      else if(strcmp(data, ":GD#") == 0)
        Serial.printf("%c%02d:%02d:%02d#", (Data.Telescope.Declination.DMS.D < 0) ? '-' : '+', abs(Data.Telescope.Declination.DMS.D), Data.Telescope.Declination.DMS.M, Data.Telescope.Declination.DMS.S, Data.Telescope.Declination.DMS.SS);

      else if(strcmp(data, ":Q#") == 0)
      {
        //None
      }
      else if(strcmp(data, ":Gg#") == 0)   //LAtitude
      {
        Serial.printf("%d#", Data.Gps.lat);  //converter para DDM
      }
      else if(strcmp(data, ":D#") == 0)   //
      {
        Serial.printf("%d#", 0x7F);
      }
      
      else if(strcmp(data, ":GVP#") == 0)    //Telescope Name
      {
        Serial.print("USJT Telescope#");
      }
      else if(strcmp(data, ":GVN#") == 0)    //Fw number
      {
        Serial.printf("%s#", _ver);
      }
      else if(strcmp(data, ":GVD#") == 0)    //Fw date
      {
        Serial.print("#");
      }
      else if(strcmp(data, ":GVT#") == 0)    //Fw time
      {
        Serial.print("#");
      }
      else if(strstr(data, ":Sr") )    //AR
      {
        taskENTER_CRITICAL(&myMutex);
        Data.Stellarium.RightAscension.HMS.H = atoi(&data[3]);
        Data.Stellarium.RightAscension.HMS.M = atoi(&data[6]);
        Data.Stellarium.RightAscension.HMS.S = atoi(&data[9]);
        Data.Stellarium.RightAscension.DT = Convert_HMS_DT(Data.Stellarium.RightAscension.HMS.H, Data.Stellarium.RightAscension.HMS.M, Data.Stellarium.RightAscension.HMS.S) + (now.tv_usec * 1e-6 * 2.7777777777777776e-7);
        taskEXIT_CRITICAL(&myMutex);
        Serial.print('1');
      }
      else if(strstr(data, ":Sd") )    //DEC
      {
        taskENTER_CRITICAL(&myMutex);
        Data.Stellarium.Declination.DMS.D = atoi(&data[3]);
        Data.Stellarium.Declination.DMS.M = atoi(&data[7]);
        Data.Stellarium.Declination.DMS.S = atoi(&data[10]);
        Data.Stellarium.Declination.DD = Convert_DEC_DMStoDeg(Data.Stellarium.Declination.DMS.D, Data.Stellarium.Declination.DMS.M, Data.Stellarium.Declination.DMS.S) + (now.tv_usec * 1e-6 * 2.7777777777777776e-7);
        taskEXIT_CRITICAL(&myMutex);
        Serial.print('1'); //ACK
        
      }
      else if((strcmp(data, ":Me#") == 0) || (strcmp(data, ":Mn#") == 0) || (strcmp(data, ":Ms#") == 0) || (strcmp(data, ":Mw#") == 0))
      {
        Data.Control.Stellarium.Mode = 0x01;
      }

      else if(strcmp(data, ":MS#") == 0)  //Slew
      {
        
        Serial.print('0');    //0 Slew is Possible
                              //1<string># Object Below Horizon w/string message
                              //2<string># Object Below Higher w/string message
        Data.Control.Stellarium.Mode = 0x01;
      }
      else if(strcmp(data, ":CM#") == 0)  //Syncornize
      {
        Serial.print("#");    
      }
    }
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
void TaskMain( void *pvParameters __attribute__((unused)) ){
  
  static unsigned long previousMillis = 0; //
  static unsigned long previousMillisDisp = 0; //
 
  for(;;){

    if ((unsigned long)(xTaskGetTickCount() - previousMillis) >= 50)
    {
      esp_task_wdt_init(30, false);
      Data.ADC.Joystick_X = analogRead(ADC_JOY_X_pin);
      Data.ADC.Joystick_Y = analogRead(ADC_JOY_Y_pin);
      Data.ADC.VUSB = analogRead(ADC_VUSB_pin);
      Data.ADC.VBat = analogRead(ADC_VBAT_pin);

      //Get data from IMU sensor
      Gyro_Read();
      //Update special data timers
      Convert_UTC_to_Julian();
      //Convert positions of Telescope to Stellarium
      AltAz_to_RaDec(Data.Telescope.Altitude.DD, Data.Telescope.Azimuth.DD, &Data.Telescope.RightAscension.DT, &Data.Telescope.Declination.DD);
      //Convert positions of Stellarium to Telescope
      RaDec_to_AltAz();
      //Update informations of display
      Display_Update();
      previousMillis = xTaskGetTickCount();
    }
    else if ((unsigned long)(xTaskGetTickCount() - previousMillisDisp) >= 250)
    {
      Display_Update();
      previousMillisDisp = xTaskGetTickCount();
    }
    Check_Buttons();
 
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

void TaskMotor( void *pvParameters __attribute__((unused)) )
{
  static unsigned long lastTime = 0; //delay to update position
  for(;;)
  {
    esp_task_wdt_init(60, false);
    //Calcule delta of position
    //Azimuth
    Data.Telescope.deltaAzimuthTarget = Data.Telescope.Azimuth.DD - Data.Stellarium.Azimuth.DD;
    if(Data.Telescope.deltaAzimuthTarget > 180.0)
      Data.Telescope.deltaAzimuthTarget -= 360.0;
    else if (Data.Telescope.deltaAzimuthTarget < -180.0)
      Data.Telescope.deltaAzimuthTarget += 360.0;
    //Altitude
    Data.Telescope.deltaAltitudeTarget = Data.Stellarium.Altitude.DD - Data.Telescope.Altitude.DD;

    if(Data.Control.Joystick)
      controlJoystick();

    //Initial condition to go the target
    else if(Data.Control.Stellarium.Mode == 0x02)
    {
      if ( xSemaphoreTake( xI2CSemaphore, ( TickType_t ) 50 ) == pdTRUE )
      {
        ioport.digitalWrite(exp_io_MTR_AZM_MODE2_pin, LOW); //config to 8 microsteps
        ioport.digitalWrite(exp_io_MTR_ALT_MODE2_pin, LOW); //config to 8 microsteps
        MTR_AZM.setMicrostep(8);
        MTR_ALT.setMicrostep(8);
        xSemaphoreGive( xI2CSemaphore ); // Now free or "Give" the Serial Port for others.
      }
      MTR_AZM.setRPM(60);
      MTR_ALT.setRPM(60);
    
      digitalWrite(SOURCE_15V_EN_pin, HIGH); //enable

      MTR_ALT.enable(); // energize coils - the motor will hold position
      MTR_AZM.enable(); // energize coils - the motor will hold position

      controller.rotate(Data.Telescope.deltaAzimuthTarget , Data.Telescope.deltaAltitudeTarget);

      if ( xSemaphoreTake( xI2CSemaphore, ( TickType_t ) 50 ) == pdTRUE )
      {
        ioport.digitalWrite(exp_io_MTR_AZM_MODE2_pin, HIGH); //config to 32 microsteps
        ioport.digitalWrite(exp_io_MTR_ALT_MODE2_pin, HIGH); //config to 32 microsteps
        MTR_AZM.setMicrostep(32);
        MTR_ALT.setMicrostep(32);
        xSemaphoreGive( xI2CSemaphore ); // Now free or "Give" the Serial Port for others.
      }

      MTR_AZM.setRPM(30); //180
      MTR_ALT.setRPM(30);

      Data.Control.Stellarium.Mode = 0x03;
      lastTime = xTaskGetTickCount();
    }
    else if (Data.Control.Stellarium.Mode == 0x03)
    {
      if(xTaskGetTickCount() - lastTime > 100)
      {
        //if(Data.Telescope.deltaAzimuthTarget || Data.Telescope.deltaAltitudeTarget != 0.00)
          controller.rotate(Data.Telescope.deltaAzimuthTarget , Data.Telescope.deltaAltitudeTarget);
        lastTime = xTaskGetTickCount();
      }
    }
    else if (!Data.Control.Joystick && digitalRead(SOURCE_15V_EN_pin))
    {
      MTR_ALT.disable(); // pause and allow the motor to be moved by hand
      MTR_AZM.disable(); // pause and allow the motor to be moved by hand
      digitalWrite(SOURCE_15V_EN_pin, LOW); //disable
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------
int xaxisState = LOW; //state of x axis stepper pulse set to ON
int yaxisState = LOW; //state of y axis stepper pulse set to ON
//
int XStepDelay;
int YStepDelay;
//
//
const long xhigh = 9; //interval to pulse X axis at slowest speed, change to fit application
const long xlow = 1; //interval to pulse X axis at highest speed, change to fit application
//
const long yhigh = 9; //interval to pulse Y axis at slowest speed, change to fit application
const long ylow = 1; //interval to pulse Y axis at highest speed, change to fit application
//
//
unsigned long previousMillisX = 0; //
unsigned long previousMillisY = 0; //

void controlJoystick(void)
{
  static int moveStepsX;
  static int moveStepsY;
  static int AZMspeed = abs(map(Data.ADC.Joystick_X, 523, 1023, 0, 300));
  static int ALTspeed = abs(map(Data.ADC.Joystick_Y, 523, 1023, 0, 300));
  static int AZMsteps = 0;
  static int ALTsteps = 0;

  
  if ((Data.ADC.Joystick_X >= 1900 || Data.ADC.Joystick_X <= 1500) /*&& ((unsigned long)(xTaskGetTickCount() - previousMillisX) >= XStepDelay)*/)
  {
      MTR_AZM.enable();
    ALTspeed = abs(map(Data.ADC.Joystick_X, 523, 1023, 0, 300));
    if(Data.ADC.Joystick_X > 1900)  //Up
    {
      XStepDelay = map(Data.ADC.Joystick_X, 1900, 4095, xlow, xhigh);
      //MTR_AZM.setRPM(XStepDelay);
      moveStepsX = 300;
    }
    else if(Data.ADC.Joystick_X < 1500)  //Down
    {
      XStepDelay = map(Data.ADC.Joystick_X, 0, 1500, -xhigh, -xlow);
      //MTR_AZM.setRPM(XStepDelay);
      moveStepsX = -300;
      
    }
  }
  else
  {
    XStepDelay = moveStepsX = 0;
    MTR_AZM.startBrake();
    MTR_AZM.stop();
    MTR_AZM.disable();
  }

  if ((Data.ADC.Joystick_Y >= 1900 || Data.ADC.Joystick_Y <= 1500) /*&& ((unsigned long)(xTaskGetTickCount() - previousMillisY) >= YStepDelay)*/)
  {
    MTR_ALT.enable();
    if(Data.ADC.Joystick_Y > 1900)  //Right
    {
      YStepDelay = map(Data.ADC.Joystick_Y, 1900, 4095, -ylow, -yhigh);
      //MTR_ALT.setRPM(YStepDelay);
      moveStepsY = -30;
       MTR_ALT.startRotate(-900);
    }
    else if(Data.ADC.Joystick_Y < 1500)  //Left
    {
      YStepDelay = map(Data.ADC.Joystick_Y, 0, 1500, yhigh, ylow);
      //MTR_ALT.setRPM(YStepDelay);
      moveStepsY = 30;
       MTR_ALT.startRotate(900);

    }
  }
  else
  {
    YStepDelay = moveStepsY = 0;
    MTR_ALT.startBrake();
    MTR_ALT.stop();
    MTR_ALT.disable();
  }


  controller.rotate(XStepDelay, YStepDelay);
  MTR_ALT.nextAction();
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
void Check_Buttons()
{
  //Buttons Read
  if (button[0].pressed_short) 
  {
    button[0].pressed_short = false;
    Data.Display.Menu ++;
  }
  else if (button[1].pressed_short) 
  {
    button[1].pressed_short = false;
    Data.Display.SubMenu ++;
  }
  else if (button[2].pressed_short) 
  {
    button[2].pressed_short = false;
    //Data.Display.SubMenu ++;
  }
  else if (button[3].pressed_short) 
  {
    button[3].pressed_short = false;
    Data.Control.Stellarium.SerialBle = ~Data.Control.Stellarium.SerialBle;
  }
  else if (button[4].pressed_short) 
  {
    button[4].pressed_short = false;

    if(Data.Control.Stellarium.Mode > 0)
    {
      Data.Control.Stellarium.Mode = 0;
      digitalWrite(SOURCE_15V_EN_pin, LOW);

      MTR_AZM.disable(); //Disable
      MTR_ALT.disable(); //Disable
    }
    else
      Data.Control.Joystick = ~Data.Control.Joystick;

    if(Data.Control.Joystick) //Enable
    {
      ledcWrite(0, 5);
      ledcWrite(1, 5);
      ledcWrite(2, 5);
      digitalWrite(SOURCE_15V_EN_pin, HIGH); //enable

      MTR_AZM.enable();  //Enable
      MTR_ALT.enable();  //Enable

      Data.Control.Stellarium.Mode = 0x00;
    }
    else  //disable
    {
      ledcWrite(0, 0);
      ledcWrite(1, 0);
      ledcWrite(2, 0);
      digitalWrite(SOURCE_15V_EN_pin, LOW);

      MTR_AZM.disable(); //Disable
      MTR_ALT.disable(); //Disable
      Data.Control.Stellarium.Mode = 0x00;
    }
  }
  if (button[3].pressed_long) 
  {
    button[3].pressed_long = false;
    digitalWrite(SOURCE_15V_EN_pin, LOW);
    digitalWrite(SOURCE_3V3_EN_pin, LOW);
    esp_deep_sleep_start();
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------
void Convert_UTC_to_Julian()
{

  uint32_t d, m ,y, H, M, S;
  double jy, jm, centry;   // scratch
  time_t now = time(0);
	tm* localtm = localtime(&now);
  
  d = localtm->tm_mday;
  m = localtm->tm_mon + 1;
  y = localtm->tm_year + 1900;
  H = localtm->tm_hour;
  M = localtm->tm_min;
  S = localtm->tm_sec;
  //Convert_UT(d, m, y, H, M, S);

  struct timeval start;
  gettimeofday(&start, NULL);

  Data.Time.UTC = now + start.tv_usec * 1e-6;
  Data.Time.TT = Data.Time.UTC  + (-37)/*leap seconds added to UTC to date*/ + (-13) /*diff time GPS*/ + 32.184;
  Data.Time.JD = floor(365.25 * y) + floor(30.6001 * ( m + 1)) + d + (H + (M/60) + (S/3600))/24 + 1720981.5;


	if (m > 2)	// jy is the year, jm is (month + 1), eg. March = 4
	{
		jy = y;
		jm = m + 1;
	}
	else	// jy is the year less 1, jm is (month + 13), eg. January = 14, February = 15
	{
		jy = y - 1;
		jm = m + 13;
	}

	double intgr = (floor)(365.25 * jy) + (floor)(30.6001 * jm ) + d + 1720995;
	/*	Simple 'every fourth year is a leap year' is applied to bump (intgr) according to the
		year.  The 30.6001 multiplier is a clever system that indexes the correct number of
		days for complete months that have occurred since 1st March.  Then add the day-of-month.
		1720995 is a frig factor that adjusts (intgr) to zero for 4713-01-01 BCE
		The fact that 1720995 is JD for 0002-10-30 BCE seems to be irrelevant! */

	/*  Check to see if we need to be in the Gregorian calendar: since d !> 31, we can test by
		assuming all months have 31 days, so set gregcal to be (1582*12 + 10) months + 15 days */
	double gregcal = 588829;	// = 15 + 31*(10 + 12*1582)
	if (d + 31*(m + 12*y) >= gregcal)
	{
		centry = (floor)(0.01 * jy);
			/*	centry will always be the century part of the year
				UNLESS Jan or Feb in a centennial year */
		intgr += 2 - centry + (floor)(0.25 * centry) - (floor)(0.025 * centry);
			/*	The middle term above corrects for the 400 year rule
				and the last for the 4000 year rule (added by SGS) */
	}

	// correct for half-day offset
	double  dayfrac = H/24.0 - 0.5;
	if (dayfrac < 0.0)
	{
		dayfrac += 1.0;
		--intgr;
	}

	// now set the fraction of a day
	double frac = dayfrac + (M + (S/60.0)) / (60.0*24.0);
		  
	// round to nearest 10ms
	double jd0 = (intgr + frac) * 10000000;
	double jd = floor(jd0);
	if ((jd0 - jd) > 0.5)
		++jd;

  //JD days
  double dwhole = 367.0 * y -(floor)(7*(y+(floor)((m+9)/12))/4)+(floor)(275*m/9)+(double)d-730531.5;
  double dfrac = (double)(H + (double)M/60 + (double)S/3600)/24;

  double jd_days = dwhole + dfrac;

  Data.Time.JD = jd/10000000;

  Data.Time.JD2000 = Data.Time.JD - 2451545.0;
  Data.Time.MJD = Data.Time.JD - 2400000.5;

  //double T = Data.Time.JD2000 / 36525;
  //Data.Time.GMST = 24110.54841 + (8640184.812866 * T) + (0.093104 * pow(T, 2)) - (0.0000062 * pow(T, 3));


  //https://caps.gsfc.nasa.gov/simpson/software/jd2gmst_f90.txt
  //convert JD to Julian centuries
  double T = Data.Time.JD2000/36525.0;
  //GMST (deg)
  double GMST = 280.46061837 + 360.98564736629 * Data.Time.JD2000 + 0.000387933 * pow(T, 2) - pow(T, 3) / 38710000;
  //reduce GMST to range 0-360 deg
  GMST = fmod(GMST, 360.0);
  if(GMST < 0)
    GMST += 360.0;

  //convert GMST to hours of arc
  Data.Time.GMST = GMST / 15.0;
  
  double LST = Data.Time.GMST + (Data.Gps.lon/15.0);
  LST = fmod(LST, 24.0);
   if(LST < 0)
     LST += 24.0;
  Data.Time.LST = LST;

  double arHH, arMM, arSS;
  Convert_RA_to_HMS(Data.Time.LST, &arHH, &arMM, &arSS);
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
void RaDec_to_AltAz(void)
{
  double Sin_LAT, Sin_DEC, Sin_HA;
  double Cos_LAT, Cos_DEC, Cos_HA;

  double ra =  Data.Time.LST - Data.Stellarium.RightAscension.DT;
  if (ra < 0.0)
    ra += 24.0;

  Sin_LAT = sin(deg2rad((Data.Gps.lat)));   //latitude
  Cos_LAT = cos(deg2rad((Data.Gps.lat)));   //latitude
  Sin_DEC = sin(deg2rad(Data.Stellarium.Declination.DD));  // declinação
  Cos_DEC = cos(deg2rad(Data.Stellarium.Declination.DD));  // declinação
  Sin_HA = sin(deg2rad(ra * 15.0));     //angule hour to degree
  Cos_HA = cos(deg2rad(ra * 15.0));     //angule hour to degree


  //https://idlastro.gsfc.nasa.gov/ftp/pro/astro/hadec2altaz.pro
  double x = - Cos_HA * Cos_DEC * Sin_LAT + Sin_DEC * Cos_LAT;
  double y = - Sin_HA * Cos_DEC;
  double z = Cos_HA * Cos_DEC * Cos_LAT + Sin_DEC * Sin_LAT;
  double r = sqrt(pow(x,2) + pow(y,2));

  double Azm = rad2deg(atan2(y, x));
  double Alt = rad2deg(atan2(z, r));
  if (Azm < 0)
    Azm = Azm + 360.0;

  Data.Stellarium.Azimuth.DD = Azm;
  Data.Stellarium.Altitude.DD = Alt;

  convert_DD_2_DMS(Data.Stellarium.Altitude.DD, &Data.Stellarium.Altitude.DMS.D, &Data.Stellarium.Altitude.DMS.M, &Data.Stellarium.Altitude.DMS.S, &Data.Stellarium.Altitude.DMS.SS);
  convert_DD_2_DMS(Data.Stellarium.Azimuth.DD, &Data.Stellarium.Azimuth.DMS.D, &Data.Stellarium.Azimuth.DMS.M, &Data.Stellarium.Azimuth.DMS.S, &Data.Stellarium.Azimuth.DMS.SS);

  //Byte of control for move to new position received after finish conversions necessaries
  if(Data.Control.Stellarium.Mode == 0x01)   //Sinilize data is updated in Fast mode 
    Data.Control.Stellarium.Mode = 0x02;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
//Convert Alt/Az to RA and Dec (J2000)
void AltAz_to_RaDec(double Altitude, double Azimuth, double *RightAscension, double *Declination)
{
  double HA, RA, DEC_rad, Alt_rad, Az_rad, Lat_rad;
  double Alt_sin, Alt_cos, Alt_tan, Az_sin, Az_cos, Lat_sin, Lat_cos, DEC_sin, DEC_cos;

  Alt_rad = deg2rad(Altitude);
  Az_rad  = deg2rad(Azimuth);
  Lat_rad = deg2rad((Data.Gps.lat));

  Alt_sin = sin(Alt_rad);
  Alt_cos = cos(Alt_rad);
  Alt_tan = tan(Alt_rad);
  Az_sin  = sin(Az_rad);
  Az_cos  = cos(Az_rad);
  Lat_sin = sin(Lat_rad);
  Lat_cos = cos(Lat_rad);


  //https://idlastro.gsfc.nasa.gov/ftp/pro/astro/altaz2hadec.pro
  DEC_rad = asin((Lat_sin * Alt_sin) + (Lat_cos * Alt_cos * Az_cos));
  *Declination  = rad2deg( DEC_rad );
  
  HA = rad2deg(atan2( (-Az_sin  * Alt_cos), (-Az_cos * Lat_sin * Alt_cos) + (Alt_sin * Lat_cos)));
  
  double ha = fmod((Data.Time.LST * 15) + 360 - HA, 360);
  RA = ha / 15.0;

  *RightAscension = RA;

  convert_DD_2_DMS(Data.Telescope.Declination.DD, &Data.Telescope.Declination.DMS.D, &Data.Telescope.Declination.DMS.M, &Data.Telescope.Declination.DMS.S, &Data.Telescope.Declination.DMS.SS);
  convert_DD_2_DMS(Data.Telescope.RightAscension.DT, &Data.Telescope.RightAscension.HMS.H, &Data.Telescope.RightAscension.HMS.M, &Data.Telescope.RightAscension.HMS.S, &Data.Telescope.RightAscension.HMS.SS);
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
void Gyro_Read(){
  static unsigned long lastTime = 0;
  static unsigned long lastTimeFreq = 0;
  static unsigned long lastFreq = 0;
  imu::Vector<3> euler;
  imu::Vector<3> accel;
  imu::Vector<3> mag;
  imu::Vector<3> gyr;
  imu::Vector<3> laccel;
  imu::Vector<3> grav;
  imu::Quaternion quat;


  if ( xSemaphoreTake( xI2CSemaphore, ( TickType_t ) 50 ) == pdTRUE )
  {
    euler  = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    if(xTaskGetTickCount() - lastTimeFreq > 1000)
    {
      Data.Inertial.Freq = lastFreq;
      lastFreq = 0;
    }
    lastFreq ++;
    lastTimeFreq = xTaskGetTickCount();
    xSemaphoreGive( xI2CSemaphore ); // Now free or "Give" the Serial Port for others.
  }


  //taskENTER_CRITICAL(&myMutex);
  Data.Inertial.Euler.X = euler.x();
  //https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
  double DecMag = Data.Inertial.Euler.X  - 21.65;
  if(DecMag < 0.0)
    DecMag += 360.0;
  Data.Telescope.Azimuth.DD = DecMag;
  Data.Inertial.Euler.Y = euler.y();
  Data.Inertial.Euler.Z = Data.Telescope.Altitude.DD  = euler.z(); 

  //Data.Inertial.EulerIIR.X = lp.filter(Data.Telescope.Azimuth.DD);
  //Data.Inertial.EulerIIR.Z = lp.filter(Data.Telescope.Altitude.DD);


  //taskEXIT_CRITICAL(&myMutex); 

  if(xTaskGetTickCount() - lastTime  > 2000)
  {
    if ( xSemaphoreTake( xI2CSemaphore, ( TickType_t ) 50 ) == pdTRUE )
    {
      accel  = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      mag    = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      gyr    = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      laccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      grav   = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
      quat   = bno.getQuat();

      bno.getCalibration(&Data.Inertial.Calibration.SYS, &Data.Inertial.Calibration.GYR, &Data.Inertial.Calibration.ACC, &Data.Inertial.Calibration.MAG);
      Data.Inertial.Temperature = bno.getTemp();
      lastTime = xTaskGetTickCount();
      xSemaphoreGive( xI2CSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    
    Data.Inertial.Accelerometer.X = accel.x();
    Data.Inertial.Accelerometer.Y = accel.y();
    Data.Inertial.Accelerometer.Z = accel.z();

    Data.Inertial.Magnetometer.X = mag.x();
    Data.Inertial.Magnetometer.Y = mag.y();
    Data.Inertial.Magnetometer.Z = mag.z();

    Data.Inertial.Gyroscope.X = gyr.x();
    Data.Inertial.Gyroscope.Y = gyr.y();
    Data.Inertial.Gyroscope.Z = gyr.z();

    Data.Inertial.LinearAccel.X = laccel.x();
    Data.Inertial.LinearAccel.Y = laccel.y();
    Data.Inertial.LinearAccel.Z = laccel.z();

    Data.Inertial.Gravity.X = grav.x();
    Data.Inertial.Gravity.Y = grav.y();
    Data.Inertial.Gravity.Z = grav.z();

    Data.Inertial.Quarternion.W = quat.w();
    Data.Inertial.Quarternion.X = quat.x();
    Data.Inertial.Quarternion.Y = quat.y();
    Data.Inertial.Quarternion.Z = quat.z();
  }
  convert_DD_2_DMS(Data.Telescope.Azimuth.DD, &Data.Telescope.Azimuth.DMS.D, &Data.Telescope.Azimuth.DMS.M, &Data.Telescope.Azimuth.DMS.S, &Data.Telescope.Azimuth.DMS.SS);
  convert_DD_2_DMS(Data.Telescope.Altitude.DD, &Data.Telescope.Altitude.DMS.D, &Data.Telescope.Altitude.DMS.M, &Data.Telescope.Altitude.DMS.S, &Data.Telescope.Altitude.DMS.SS);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------------------------------------------
void drawIcons(uint8_t bat_level)
{
  uint8_t bat_st1, bat_st2, bat_pg, source_err;
  static uint32_t lastShowIcon;
  static char showIcon = 0;
  time_t now = time(0) - 3600 * 3;

  //get time from system
	tm* localtm = localtime(&now);

  //Read IO's data to menu
  if ( xSemaphoreTake( xI2CSemaphore, ( TickType_t ) 50 ) == pdTRUE )
  {
    bat_st1 = ioport.digitalRead(exp_io_CHARG_ST1_pin);
    bat_st2 = ioport.digitalRead(exp_io_CHARG_ST2_pin);
    bat_pg  = ~ioport.digitalRead(exp_io_CHARG_PG_pin);
    xSemaphoreGive( xI2CSemaphore ); // Now free or "Give" the Serial Port for others.
  }

  source_err  = digitalRead(SOURCE_DC_ERR_pin);

  if(xTaskGetTickCount() - lastShowIcon >= 800)
  {
    lastShowIcon = xTaskGetTickCount();
    showIcon = ~showIcon;
  }

  //Print Time
  display.setCursor(0,3);
  display.printf("%02d", localtm->tm_hour);
  display.setCursor(10,3);
  display.printf(":");
  display.setCursor(14,3);
  display.printf("%02d", localtm->tm_min);
  display.setCursor(24,3);
  display.printf(":");
  display.setCursor(28,3);
  display.printf("%02d", localtm->tm_sec);

  display.drawFastHLine(0, 13, 128, SSD1306_WHITE);   //Draw lines

  //Icon 1 - Icon Manual / Auto
  if(Data.Control.Joystick)
    display.drawBitmap(OFFSET_MN_ICON_1, 0, icon_man, 12, 12, 1);
  else if(Data.Control.Stellarium.Mode)
    display.drawBitmap(OFFSET_MN_ICON_1, 0, icon_aut, 12, 12, 1);

  //Icon 2 - Icon BLE / SERIAL
  if(Data.Control.Stellarium.SerialBle)
    display.drawBitmap(OFFSET_MN_ICON_2, 0, icon_ble, 12, 12, 1);
  else
    display.drawBitmap(OFFSET_MN_ICON_2, 0, icon_serial, 12, 12, 1);

  //Icon 3 - Bussola
  if(Data.Inertial.Calibration.MAG >= 2)  
    display.drawBitmap(OFFSET_MN_ICON_3, 0, icon_bussola, 12, 12, 1);
  else if(showIcon)
    display.drawBitmap(OFFSET_MN_ICON_3, 0, icon_bussoless, 12, 12, 1);

  //Icon 4 - Targert
  if(!Data.Control.Stellarium.Mode);
  else if(Data.Telescope.deltaAzimuthTarget < 0.001 && Data.Telescope.deltaAltitudeTarget < 0.001)
    display.drawBitmap(OFFSET_MN_ICON_4, 0, icon_target, 12, 12, 1);
  else if(showIcon)
    display.drawBitmap(OFFSET_MN_ICON_4, 0, icon_targetless, 12, 12, 1);

  //Icon 5 - Icon GPS
  if(!Data.Gps.flags.bits.gnssOK)
    display.drawBitmap(OFFSET_MN_ICON_5, 0, icon_gpsless, 12, 12, 1);
  else if(Data.Gps.flags.bits.gnssFixOK)
    display.drawBitmap(OFFSET_MN_ICON_5, 0, icon_gps, 12, 12, 1);
  else
    if(showIcon)
      display.drawBitmap(OFFSET_MN_ICON_5, 0, icon_gps, 12, 12, 1);

  //Icon 6 - Print Source Icon
  if(source_err && !bat_st1 && !bat_st2 && !bat_pg)             //No imput power
    display.drawBitmap(OFFSET_MN_ICON_6, 0, icon_batless, 12, 12, 1);
  else if(!source_err)                                          //Power Supply
    display.drawBitmap(OFFSET_MN_ICON_6, 0, icon_source, 12, 12, 1);
  else if(Data.ADC.VUSB > 2000)                          //USB
    display.drawBitmap(OFFSET_MN_ICON_6, 0, icon_usb, 12, 12, 1);  

  //Icon 7 - Print Battery Icon
  if(!bat_st1 && !bat_st2 && bat_pg)                            //No Battery
    display.drawBitmap(OFFSET_MN_ICON_7, 0, icon_batless, 12, 12, 1);
  else if(!bat_st1 && bat_st2 && bat_pg)                        //Charging Complete
    if(showIcon)
      display.drawBitmap(OFFSET_MN_ICON_7, 0, icon_bat100, 12, 12, 1);
  else if(bat_st1 && !bat_st2 && bat_pg)                        //Charging
    display.drawBitmap(OFFSET_MN_ICON_7, 0, icon_charging, 12, 12, 1);
  else if(bat_level <= 5)                                       //Battery 0%
    display.drawBitmap(OFFSET_MN_ICON_7, 0, icon_bat0, 12, 12, 1);
  else if(bat_level <= 25)                                      //Battery 25%
    display.drawBitmap(OFFSET_MN_ICON_7, 0, icon_bat25, 12, 12, 1);
  else if(bat_level <= 50)                                      //Battery 50%
    display.drawBitmap(OFFSET_MN_ICON_7, 0, icon_bat50, 12, 12, 1);
  else if(bat_level <= 75)                                      //Battery 75%
    display.drawBitmap(OFFSET_MN_ICON_7, 0, icon_bat75, 12, 12, 1);
  else if(bat_level > 75)                                       //Battery 100%
    display.drawBitmap(116, 0, icon_bat100, 12, 12, 1);
}

void drawButtons(void)
{
  //Draw Buttons
  display.drawBitmap(OFFSET_MN_BT_1, 49, bt_menu, 24, 14, 1);
  display.drawBitmap(OFFSET_MN_BT_2, 49, bt_submenu, 24, 14, 1);
  display.drawBitmap(OFFSET_MN_BT_3, 49, bt_change, 24, 14, 1);
  display.drawBitmap(OFFSET_MN_BT_4, 49, bt_bleserial, 24, 14, 1);

}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
void Display_Update(){

  /* Menus and Submenus:
     0 - Logo
     1 - BNO055
        1.1 - Euler
        1.2 - Magnetometro
        1.3 - Accelerometer
        1.4 - 
     2 - GPS
     3 - 
  */
  #define MAX_MENUS 6
  uint8_t MAX_SUBMENU[MAX_MENUS] = {1, 7, 2, 2, 2, 2};
  float vUSB, vBAT;
  int8_t batLevel;


    //Initial verifications
    if(Data.Display.Menu > MAX_MENUS)
    {
      Data.Display.Menu = 0;
      Data.Display.SubMenu = 0;
    }

    if(Data.Display.SubMenu > MAX_SUBMENU[Data.Display.Menu])
      Data.Display.SubMenu = 0;

    display.clearDisplay();
    //display.cp437(true);

    vUSB      = ((Data.ADC.VUSB * VREF) / RESOLUTION) * ((R1 + R2) / R2);
    vBAT      = ((Data.ADC.VBat * VREF) / RESOLUTION) * ((R1 + R2) / R2) * 1000;
    batLevel  = battery_voltage_to_percent[(uint8_t)((vBAT - BATTERY_MEAS_MIN_LEVEL + (VOLTAGE_TO_PERCENT_DELTA >> 1)) / VOLTAGE_TO_PERCENT_DELTA)];
    batLevel  = (batLevel > 100? 100: (batLevel < 0? 0 : batLevel));
      
    drawIcons(batLevel);
    drawButtons();

    display.setCursor(0,16);
    if(Data.Display.Menu == 0)  //Buttons
    {
      if(Data.Display.SubMenu == 0)
      {
        display.drawBitmap(1, 15, logo_mini, 128, 34, 1);
      }
      if(Data.Display.SubMenu == 1)
      {
        display.printf("Azm: %2.6f%c\n", Data.Telescope.deltaAzimuthTarget, char(9));
        display.printf("Alt: %2.6f%c\n", Data.Telescope.deltaAltitudeTarget, char(9));
        display.printf("\n");
        display.printf("Diff Angles - Delta");
      }
      display.drawBitmap(1, 49, mode_measure, 32, 14, 1);
    }
    else if(Data.Display.Menu == 1)  //Inertial
    {
      if(Data.Display.SubMenu == 0)
      {
        display.printf("X: % 3.6f%c (Az)\n", Data.Inertial.Euler.X, char(9)); //Heading
        display.printf("Y: % 3.6f%c\n", Data.Inertial.Euler.Y, char(9));
        display.printf("Z: % 3.6f%c (Alt)\n", Data.Inertial.Euler.Z, char(9));
        display.printf("Euler Angles");
      }
      if(Data.Display.SubMenu == 1)
      {
        display.printf("Sys: %d - Gyr: %d\n", Data.Inertial.Calibration.SYS, Data.Inertial.Calibration.GYR);
        display.printf("Acc: %d - Mag: %d\n", Data.Inertial.Calibration.ACC, Data.Inertial.Calibration.MAG);
        display.printf("Temp: % 3d%c\n", Data.Inertial.Temperature, char(9));
        display.printf("Calibration");
      }

      if(Data.Display.SubMenu == 2)
      {
        display.printf("X: % 3.6fm/s2\n", Data.Inertial.Gravity.X);
        display.printf("Y: % 3.6fm/s2\n", Data.Inertial.Gravity.Y);
        display.printf("Z: % 3.6fm/s2\n", Data.Inertial.Gravity.Z);
        display.printf("Gravity");
      }
      if(Data.Display.SubMenu == 3)
      {
        display.printf("X: % 3.6f%c/s\n", Data.Inertial.Gyroscope.X, char(9));
        display.printf("Y: % 3.6f%c/s\n", Data.Inertial.Gyroscope.Y, char(9));
        display.printf("Z: % 3.6f%c/s\n", Data.Inertial.Gyroscope.Z, char(9));
        display.printf("Gyroscope");
      }
      if(Data.Display.SubMenu == 4)
      {
        display.printf("X: % 3.6fuT\n", Data.Inertial.Magnetometer.X);
        display.printf("Y: % 3.6fuT\n", Data.Inertial.Magnetometer.Y);
        display.printf("Z: % 3.6fuT\n", Data.Inertial.Magnetometer.Z);
        display.printf("Magnetometer");
      }
      if(Data.Display.SubMenu == 5)
      {
        display.printf("X: % 3.6fm/s2\n", Data.Inertial.Accelerometer.X);
        display.printf("Y: % 3.6fm/s2\n", Data.Inertial.Accelerometer.Y);
        display.printf("Z: % 3.6fm/s2\n", Data.Inertial.Accelerometer.Z);
        display.printf("Accelerometer");
      }
      if(Data.Display.SubMenu == 6)
      {
        display.printf("X: % 3.6fm/s2\n", Data.Inertial.LinearAccel.X);
        display.printf("Y: % 3.6fm/s2\n", Data.Inertial.LinearAccel.Y);
        display.printf("Z: % 3.6fm/s2\n", Data.Inertial.LinearAccel.Z);
        display.printf("Linear Acceleration");
      }
      if(Data.Display.SubMenu == 7)
      {
        display.printf("W:% 2.3f X:% 2.3f\n", Data.Inertial.Quarternion.W, Data.Inertial.Quarternion.X);
        display.printf("Y:% 2.3f Z:% 2.3f\n", Data.Inertial.Quarternion.Y, Data.Inertial.Quarternion.Z);
        display.printf("\n");
        display.printf("Quarternion");
      }
      display.drawBitmap(1, 49, mode_gyr, 32, 14, 1);
    }
    else if(Data.Display.Menu == 2)  //GPS
    {
      if(Data.Display.SubMenu == 0)
      {
        display.printf("%02d/%02d/%04d\n", Data.Gps.day, Data.Gps.month, Data.Gps.year);
        display.printf("%02d:%02d:%02d\n", Data.Gps.hour, Data.Gps.min, Data.Gps.sec);
        display.printf("Lat: %3.6f%c\n", Data.Gps.lat, char(9));
        display.printf("Lon: %3.6f%c", Data.Gps.lon, char(9));
      }
      display.drawBitmap(1, 49, mode_gps, 32, 14, 1);
    }
    else if(Data.Display.Menu == 3)  //Telescope
    {
      if(Data.Display.SubMenu == 0)
      {
        display.printf("UTC:%lf\n", Data.Time.UTC);
        display.printf("TT: %lf\n", Data.Time.TT);
        display.printf("JD: %lf\n", Data.Time.JD);
      }      
      if(Data.Display.SubMenu == 1)
      {
        display.printf("J2000: %lf\n", Data.Time.JD2000);
        display.printf("GMST:  %lf\n", Data.Time.GMST);
        display.printf("LST:   %lf\n", Data.Time.LST);
      }


  //      display.printf("Az:  %02dd%02dm%02ds\n", Data.Telescope.Azimuth.DMS.D, Data.Telescope.Azimuth.DMS.M, Data.Telescope.Azimuth.DMS.S, Data.Telescope.Azimuth.DMS.SS);
  ///      display.printf("Alt: %02dd%02dm%02ds\n", Data.Telescope.Altitude.DMS.D, Data.Telescope.Altitude.DMS.M, Data.Telescope.Altitude.DMS.S, Data.Telescope.Altitude.DMS.SS);
  //      display.printf("RA:  %02dh%02dm%02ds\n", Data.Telescope.RightAscension.HMS.H, Data.Telescope.RightAscension.HMS.M, Data.Telescope.RightAscension.HMS.S, Data.Telescope.RightAscension.HMS.SS);
  //      display.printf("DEC: %02dd%02dm%02ds", Data.Telescope.Declination.DMS.D, Data.Telescope.Declination.DMS.M, Data.Telescope.Declination.DMS.S, Data.Telescope.Declination.DMS.SS);
    
      if(Data.Display.SubMenu == 2)
      {
        display.printf("Az: %4.6f\n", Data.Telescope.Azimuth.DD);
        display.printf("Alt:%4.6f\n", Data.Telescope.Altitude.DD);
        display.printf("RA: %4.6f\n", Data.Telescope.RightAscension.DT);
        display.printf("DEC:%4.6f", Data.Telescope.Declination.DD);
      }
      display.drawBitmap(1, 49, mode_tmr, 32, 14, 1);
    }

    else if(Data.Display.Menu == 4)  //Stellarium
    {
      if(Data.Display.SubMenu == 0)
      {
        display.printf("ALT: %02d.%02d.%02d\n", Data.Stellarium.Altitude.DMS.D, Data.Stellarium.Altitude.DMS.M, Data.Stellarium.Altitude.DMS.S);
        display.printf("AZ:  %02d.%02d.%02d\n", Data.Stellarium.Azimuth.DMS.D, Data.Stellarium.Azimuth.DMS.M, Data.Stellarium.Azimuth.DMS.S);
        display.printf("RA:  %02d.%02d.%02d\n", Data.Stellarium.RightAscension.HMS.H, Data.Stellarium.RightAscension.HMS.M, Data.Stellarium.RightAscension.HMS.S);
        display.printf("DEC: %02d.%02d.%02d", Data.Stellarium.Declination.DMS.D, Data.Stellarium.Declination.DMS.M, Data.Stellarium.Declination.DMS.S);
      }
      if(Data.Display.SubMenu == 1)
      {
        display.printf("ALT: %3.6f\n", Data.Stellarium.Altitude.DD);
        display.printf("AZ:  %3.6f\n", Data.Stellarium.Azimuth.DD);
        display.printf("RA:  %3.6f\n", Data.Stellarium.RightAscension.DD);
        display.printf("DEC: %3.6f", Data.Stellarium.Declination.DD);
      }
      display.drawBitmap(1, 49, mode_stl, 32, 14, 1);
    }
    else if(Data.Display.Menu == 5)  //Telescope
    {
      if(Data.Display.SubMenu == 0)
      {
        display.printf("RA_S: %4.4f\n", Data.Stellarium.RightAscension.DD);
        display.printf("DEC_S: %4.4f\n", Data.Stellarium.Declination.DD);
        display.printf("AZ_S: %4.4f\n", Data.Stellarium.Azimuth.DD);
        display.printf("ALT_S: %4.4f", Data.Telescope.Altitude.DD);
      }
      if(Data.Display.SubMenu == 1)
      {
        display.printf("AZ_S: %4.4f\n", Data.Stellarium.Azimuth.DD);
        display.printf("ALT_S: %4.4f", Data.Telescope.Altitude.DD);
      }
      display.drawBitmap(1, 49, mode_tlc, 32, 14, 1);
    }

    else if(Data.Display.Menu == 6)  //IO's
    {
      if(Data.Display.SubMenu == 0)
      {
        display.printf("VBAT: %2.2fV -%3d%%\n", vBAT / 1000, batLevel);
        display.printf("VUSB: %2.2fV\n", vUSB);
        display.printf("ADC Joystick X: %4d\n", Data.ADC.Joystick_X);
        display.printf("ADC Joystick Y: %4d", Data.ADC.Joystick_Y);
      }
      display.drawBitmap(1, 49, mode_measure, 32, 14, 1);
    }

  if ( xSemaphoreTake( xI2CSemaphore, ( TickType_t ) 50 ) == pdTRUE )
  {
    display.display();
    xSemaphoreGive( xI2CSemaphore ); // Now free or "Give" the Serial Port for others.
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
double deg2rad(double n) {
  return n * 0.0174532925199;  //M_PI/180
}

double rad2deg(double n) {
  return n * 57.2957795131;    //180/M_PI
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
//Calculate Longitude (Deg) from Right Ascension (HMS)
double Convert_HMS_DT(double Hour, double Min, double Sec)
{
  double dt;
  dt = ((abs(Hour) + (Min / 60.0) + (Sec / 3600.0)));
  return dt;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
//Calculate Longitude (Deg) from Right Ascension (HMS)
double Convert_DEC_DMStoDeg(double Deg, double Min, double Sec)
{
  double deg;
  deg = (abs(Deg) + (Min / 60.0) + (Sec / 3600.0));
  if(Deg < 0)
    deg *= -1.0;
  return deg;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
void Convert_RA_to_HMS(double Dec, double* Hour, double* Min, double* Sec)
{
  *Hour = (int)(Dec);
  *Min = (int)((Dec - (double)*Hour) * 60);
  *Sec = (int)(((((Dec) - (double)*Hour) * 60) - *Min /*/ 60*/) * 60);
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
/*void Convert_RA_DegtoHMS(double Deg, int *Hour, int *Min, float *Sec)
{
  *Hour = (int)(Deg / 15);
  *Min = (int)abs(((Deg/15) - (double)*Hour) * 60);
  *Sec = (int)abs(((((Deg/15) - (double)*Hour) / 60) - (double)*Min) / 60) * 3600;
}
void convert_DD_2_HMS(double Deg, int *Hour, int *Min, int *Sec, int *SS)
{
  *Hour = (int)(Deg / 15);
  *Min = (int)abs(((Deg/15) - (double)*Hour) * 60);
  *Sec = (int)abs(((((Deg/15) - (double)*Hour) / 60) - (double)*Min) / 60) * 3600;
}*/
//--------------------------------------------------------------------------------------------------------------------------------------------------------
//Convert Decimal Degree Coordinates to DMS (Degrees / Minutes / Seconds / Miliseconds)
void convert_DD_2_DMS(double Coord, int *Deg, int *Min, int *Sec, int *Mili)
{
  *Deg = (int)Coord;
  *Min = (int)((abs(Coord) - abs(*Deg)) * 60);
  *Sec = (int)((abs(Coord) - abs(*Deg) - ((double)(*Min) / 60)) * 3600);
  *Mili = (int)((abs(Coord) - abs(*Deg) - ((double)(*Min) / 60 )) * 3600 * 1000) % 1000;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------
int serial_Config(void)
{
  display.print("Serial");
  display.display();

  Serial.begin(9600);           //Stellarium
  SerialGPS.begin(9600);        //GPS
  SerialDebug.begin(921600);    //Console debugger

  SerialBT.begin("Telescope Service"); //Bluetooth device name
  display.println(" - OK");
  display.display();
  return 0;
}

// Função ISR
void IRAM_ATTR ISR_Button()
{
  static unsigned long last_interrupt_time_bt[5] = {0, 0, 0, 0, 0};
  unsigned long interrupt_time = xTaskGetTickCount();

  //check btn
  for(int i = 0; i < MAX_BUTTONS; i++)
  {
    button[i].state = digitalRead(button[i].PIN);
    if (button[i].state != button[i].old_state)   //changed button
    {
      button[i].old_state = button[i].state;
      if (!button[i].state)   //pressed
        last_interrupt_time_bt[i] = xTaskGetTickCount();
    
      else if (button[i].state)
      {
        if((interrupt_time - last_interrupt_time_bt[i]) < DEBOUNCE_TIME)
          break;
        else if ((interrupt_time - last_interrupt_time_bt[i]) < SHORT_TIME)
          button[i].pressed_short = true;
        else if ((interrupt_time - last_interrupt_time_bt[i]) > LONG_TIME)
          button[i].pressed_long = true;
      }
    }
  }
}

int buttons_Config(){
  display.print("Buttons");
  display.display();

  //gpio_iomux_out(0, FUNC_GPIO0_GPIO0_0, false);
  //gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT_OUTPUT );

  gpio_iomux_out(12, FUNC_MTDI_GPIO12, false);
  gpio_set_direction(GPIO_NUM_12, GPIO_MODE_INPUT_OUTPUT );

  gpio_iomux_out(13, FUNC_MTCK_GPIO13, false);
  gpio_set_direction(GPIO_NUM_13, GPIO_MODE_INPUT_OUTPUT );

  gpio_iomux_out(14, FUNC_MTMS_GPIO14, false);
  gpio_set_direction(GPIO_NUM_14, GPIO_MODE_INPUT_OUTPUT );

  gpio_iomux_out(15, FUNC_MTDO_GPIO15, false);
  gpio_set_direction(GPIO_NUM_15, GPIO_MODE_INPUT_OUTPUT );

  delay(2);

  pinMode(button[0].PIN, INPUT_PULLUP);
  pinMode(button[1].PIN, INPUT_PULLUP);
  pinMode(button[2].PIN, INPUT_PULLUP);
  pinMode(button[3].PIN, INPUT_PULLUP);
  pinMode(button[4].PIN, INPUT_PULLUP);

  attachInterrupt(button[0].PIN, ISR_Button, CHANGE);
  attachInterrupt(button[1].PIN, ISR_Button, CHANGE);
  attachInterrupt(button[2].PIN, ISR_Button, CHANGE);
  attachInterrupt(button[3].PIN, ISR_Button, CHANGE);
  attachInterrupt(button[4].PIN, ISR_Button, CHANGE);

  display.println(" - OK");
  display.display();
  return 0;
}
int led_Config(){
  display.print("Led RGB");
  display.display();
  
  ledcSetup(0, 15000, 8);
  ledcSetup(1, 15000, 8);
  ledcSetup(2, 15000, 8);

  //LED RGB
  ledcAttachPin(RGB_R_pin, 0);
  ledcAttachPin(RGB_G_pin, 1);
  ledcAttachPin(RGB_B_pin, 2);

  display.println(" - OK");
  display.display();
  return 0;
}

int gps_Config(){
  display.print("GNSS");
  display.display();
  
  //GNSS
  SerialGPS.begin(9600);
 
  display.println(" - OK");
  display.display();
  return 0;
}

// Função ISR
void IRAM_ATTR ISR_GPS_TPulse()
{
  struct tm tm;

  if(Data.Gps.valid.bits.validDate && Data.Gps.valid.bits.validTime)
  {
    time_t systemNow = time(0);

    tm.tm_year = Data.Gps.year - 1900;
    tm.tm_mon = Data.Gps.month - 1;
    tm.tm_mday = Data.Gps.day;
    tm.tm_hour = Data.Gps.hour;
    tm.tm_min = Data.Gps.min;
    tm.tm_sec = Data.Gps.sec;
    time_t gpsNow = mktime(&tm);

    if(systemNow != gpsNow)
    {
      struct timeval now = { .tv_sec = gpsNow + 1, .tv_usec = 0 };
      settimeofday(&now, NULL);
    }
  }
}


int gpio_Config(){

  pinMode(SOURCE_3V3_EN_pin, OUTPUT);
  digitalWrite(SOURCE_3V3_EN_pin, HIGH);    //Enable

  pinMode(SOURCE_15V_EN_pin, OUTPUT);
  digitalWrite(SOURCE_15V_EN_pin, LOW);    //Disable

  pinMode(GPS_TPULSE_pin, INPUT);
  attachInterrupt(GPS_TPULSE_pin, ISR_GPS_TPulse, RISING);

  pinMode(SOURCE_DC_ERR_pin, INPUT);

  //gpio_iomux_out(8, FUNC_SD_DATA1_SD_DATA1, false);
  //gpio_set_direction(GPIO_NUM_8, GPIO_MODE_OUTPUT );
  //pinMode(BUZZER_pin, OUTPUT);
  //digitalWrite(BUZZER_pin, LOW);            //Disable
  //delay(200);
  //digitalWrite(BUZZER_pin, HIGH);            //Disable

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 1);


// AD Joystick
  pinMode(ADC_JOY_X_pin, INPUT);
  pinMode(ADC_JOY_Y_pin, INPUT);

// AD Voltage
  pinMode(ADC_VUSB_pin, INPUT);
  pinMode(ADC_VBAT_pin, INPUT);

  return 0;
}

int ioexpander_Config(){
//  display.print("IO Expander");
//  display.display();

	ioport.begin();
	ioport.setClock(400000);

  ioport.pinMode(exp_io_MTR_AZM_EN_pin, OUTPUT);      //Stepper Motor
  ioport.pinMode(exp_io_MTR_AZM_DIR_pin, OUTPUT);     //Stepper Motor
  ioport.pinMode(exp_io_MTR_AZM_MODE2_pin, OUTPUT);   //Stepper Motor
  ioport.pinMode(exp_io_MTR_AZM_SLP_FLT_pin, INPUT);  //Stepper Motor
  ioport.pinMode(exp_io_MTR_ALT_EN_pin, OUTPUT);      //Stepper Motor
  ioport.pinMode(exp_io_MTR_ALT_DIR_pin, OUTPUT);     //Stepper Motor
  ioport.pinMode(exp_io_MTR_ALT_MODE2_pin, OUTPUT);   //Stepper Motor
  ioport.pinMode(exp_io_MTR_ALT_SLP_FLT_pin, INPUT);  //Stepper Motor
  ioport.pinMode(exp_io_CHARG_PG_pin, INPUT);      //Charger Li Ion
  ioport.pinMode(exp_io_CHARG_ST1_pin, INPUT);     //Charger Li Ion
  ioport.pinMode(exp_io_CHARG_ST2_pin, INPUT);     //Charger Li Ion
  ioport.pinMode(exp_io_CHARG_CE_pin, OUTPUT);     //Charger Li Ion
  ioport.pinMode(exp_io_CHARG_PROG2_pin, OUTPUT);  //Charger Li Ion
  ioport.pinMode(exp_io_CHARG_TE_pin, OUTPUT);     //Charger Li Ion
  ioport.pinMode(exp_io_DSP_RST_pin, OUTPUT);      //Display
  ioport.pinMode(exp_io_DSP_DC_pin, OUTPUT);       //Display

  ioport.digitalWrite(exp_io_DSP_DC_pin, LOW);
  ioport.digitalWrite(exp_io_DSP_RST_pin, LOW);
  ioport.digitalWrite(exp_io_CHARG_CE_pin, HIGH);
  ioport.digitalWrite(exp_io_CHARG_PROG2_pin, HIGH);
  ioport.digitalWrite(exp_io_CHARG_TE_pin, HIGH);
  ioport.digitalWrite(exp_io_MTR_AZM_EN_pin, HIGH);
  ioport.digitalWrite(exp_io_MTR_ALT_EN_pin, HIGH);

  return 0;
}

int motor_Config(){
  display.print("Stepper Motors");
  display.display();

  MTR_AZM.begin(MOTOR_AZM_RPM, MICROSTEPS);
  MTR_ALT.begin(MOTOR_ALT_RPM, MICROSTEPS);

  MTR_AZM.setEnableActiveState(LOW);  //Enable
  MTR_ALT.setEnableActiveState(LOW);  //Enable

  MTR_AZM.setSpeedProfile(MTR_AZM.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  MTR_ALT.setSpeedProfile(MTR_AZM.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);

  if ( xSemaphoreTake( xI2CSemaphore, ( TickType_t ) 50 ) == pdTRUE )
  {
    ioport.digitalWrite(exp_io_MTR_AZM_MODE2_pin, LOW); //config to 8 microsteps
    ioport.digitalWrite(exp_io_MTR_ALT_MODE2_pin, LOW); //config to 8 microsteps
    MTR_AZM.setMicrostep(8);
    MTR_ALT.setMicrostep(8);
    xSemaphoreGive( xI2CSemaphore ); // Now free or "Give" the Serial Port for others.
  }
  MTR_AZM.setRPM(60);
  MTR_ALT.setRPM(60);

  display.println(" - OK");
  display.display();
  return 0;
}

int display_Config(){
  ioport.digitalWrite(exp_io_DSP_DC_pin, LOW);
  delay(2);
  ioport.digitalWrite(exp_io_DSP_RST_pin, HIGH);
  //SSD1306_EXTERNALVCC SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { 
    ioport.digitalWrite(exp_io_DSP_RST_pin, LOW);
    return -1;
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(200); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  return 0;
}

  //Inerctial Sensor
int sensor_Config(){
  display.print("Inertial Sensor");
  display.display();

  // Initialise the sensor
  if(!bno.begin())
  {
    display.println(" - FAIL");
    display.display();
    return -1;
  }
  bno.setExtCrystalUse(true);
  
  display.println(" - OK");
  display.display();
  return 0;
}

int ble_Config(){
}

void TaskDebug( void *pvParameters __attribute__((unused)) ){
  
  for(;;){
    vTaskDelay(200/portTICK_PERIOD_MS);
    if ( xSemaphoreTake( xSerialDBGSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {

      SerialDebug.print("\033[?25l"); //set cursor invisible
      SerialDebug.print("\033[2J"); // Clear terminal

      SerialDebug.print("\033[1;10f"); //set cursor position
      SerialDebug.printf("GNSS Lat: %3.8f (deg)", Data.Gps.lat);
      SerialDebug.print("\033[2;1f"); //set cursor position
      SerialDebug.printf("Lng: %3.8f (deg)", Data.Gps.lon);
      //SerialDebug.print("\033[3;1f"); //set cursor position
      //SerialDebug.print("Lat: %02d:%02d:%02d.%03d (GMS)", latHH, latMM, latSS, latMIL);
      //SerialDebug.print("\033[4;1f"); //set cursor position

      SerialDebug.print("\033[6;0f"); //set cursor position
      SerialDebug.print("Gyroscope\r\n");
      SerialDebug.printf("H: %2.6f\r\n", Data.Telescope.Azimuth.DD);
      SerialDebug.printf("Y: %2.6f", Data.Telescope.Altitude.DD);

      SerialDebug.print("\033[60;1f"); //set cursor position
      SerialDebug.printf("Telescope Az:  % 4.6f % 3d°%02d:%02d.%03d\t", Data.Telescope.Azimuth.DD, Data.Telescope.Azimuth.DMS.D, Data.Telescope.Azimuth.DMS.M, Data.Telescope.Azimuth.DMS.S, Data.Telescope.Azimuth.DMS.SS);
      SerialDebug.printf("Alt: % 4.6f % 3d°%02d:%02d.%03d\t", Data.Telescope.Altitude.DD, Data.Telescope.Altitude.DMS.D, Data.Telescope.Altitude.DMS.M, Data.Telescope.Altitude.DMS.S, Data.Telescope.Altitude.DMS.SS);
      SerialDebug.printf("RA:  % 4.6f % 3d:%02d:%02d.%03d\t", Data.Telescope.RightAscension.DT, Data.Telescope.RightAscension.HMS.H, Data.Telescope.RightAscension.HMS.M, Data.Telescope.RightAscension.HMS.S, Data.Telescope.RightAscension.HMS.SS);
      SerialDebug.printf("DEC: % 4.6f % 3d°%02d:%02d.%03d\r\n", Data.Telescope.Declination.DD, Data.Telescope.Declination.DMS.D, Data.Telescope.Declination.DMS.M, Data.Telescope.Declination.DMS.S, Data.Telescope.Declination.DMS.SS);
      SerialDebug.printf("Delta Alt: %4.6f Az: %4.6f Mode %02d\r\n", Data.Telescope.deltaAltitudeTarget, Data.Telescope.deltaAzimuthTarget, Data.Control.Stellarium.Mode);

      SerialDebug.printf("Stellarium Az:  %4.6f %3d°%02d:%02d.%03d\t", Data.Stellarium.Azimuth.DD, Data.Stellarium.Azimuth.DMS.D, Data.Stellarium.Azimuth.DMS.M, Data.Stellarium.Azimuth.DMS.S, Data.Stellarium.Azimuth.DMS.SS);
      SerialDebug.printf("Alt: %4.6f %3d°%02d:%02d.%03d\t", Data.Stellarium.Altitude.DD, Data.Stellarium.Altitude.DMS.D, Data.Stellarium.Altitude.DMS.M, Data.Stellarium.Altitude.DMS.S, Data.Stellarium.Altitude.DMS.SS);
      SerialDebug.printf("RA:  %4.6f %3d:%02d:%02d.%03d\t", Data.Stellarium.RightAscension.DD, Data.Stellarium.RightAscension.HMS.H, Data.Stellarium.RightAscension.HMS.M, Data.Stellarium.RightAscension.HMS.S, Data.Stellarium.RightAscension.HMS.SS);
      SerialDebug.printf("DEC: %4.6f %3d°%02d:%02d.%03d\r\n", Data.Stellarium.Declination.DD, Data.Stellarium.Declination.DMS.D, Data.Stellarium.Declination.DMS.M, Data.Stellarium.Declination.DMS.S, Data.Stellarium.Declination.DMS.SS);
      SerialDebug.printf("Delta Alt: %4.6f Az: %4.6f\r\n", (Data.Stellarium.Altitude.DD -  Data.Telescope.Altitude.DD), (Data.Stellarium.Azimuth.DD -  Data.Telescope.Azimuth.DD));

      SerialDebug.print("\033[20;1f"); // L1 C1
      SerialDebug.print("Convert HaDec to AltAz:");
      SerialDebug.print("\033[21;1f"); // L1 C1
      SerialDebug.printf("Data.Stellarium.Altitude.DD (Z):       %4.6f %02d %02d:%02d.%03d", Data.Stellarium.Altitude.DD,Data.Stellarium.Altitude.DMS.D, Data.Stellarium.Altitude.DMS.M, Data.Stellarium.Altitude.DMS.S, Data.Stellarium.Altitude.DMS.SS);
      SerialDebug.print("\033[22;1f"); // L1 C1
      SerialDebug.printf("Data.Stellarium.Azimuth.DD (X):        %4.6f %02d %02d:%02d.%03d", Data.Stellarium.Azimuth.DD,Data.Stellarium.Azimuth.DMS.D, Data.Stellarium.Azimuth.DMS.M, Data.Stellarium.Azimuth.DMS.S, Data.Stellarium.Azimuth.DMS.SS);
      SerialDebug.print("\033[23;1f"); // L1 C1
      SerialDebug.printf("Data.Stellarium.Declination.DD (Z):    %4.6f %02d %02d:%02d.%03d", Data.Stellarium.Declination.DD, Data.Stellarium.Declination.DMS.D, Data.Stellarium.Declination.DMS.M, Data.Stellarium.Declination.DMS.S, Data.Stellarium.Declination.DMS.SS);
      SerialDebug.print("\033[24;1f"); // L1 C1
      SerialDebug.printf("Data.Stellarium.RightAscension.DT (X): %4.6f %02d %02d:%02d.%03d", Data.Stellarium.RightAscension.DT, Data.Stellarium.RightAscension.HMS.H, Data.Stellarium.RightAscension.HMS.M, Data.Stellarium.RightAscension.HMS.S, Data.Stellarium.RightAscension.HMS.SS);


      SerialDebug.print("\033[35;1f"); // L35 C1
      SerialDebug.printf("UTC seg: %.3f", Data.Time.UTC);  //em segundos
      SerialDebug.print("\033[36;1f"); // L37 C1
      SerialDebug.printf("TT seg:  %.6f", Data.Time.TT);  //em dias
      SerialDebug.print("\033[37;1f"); // L36 C1
      SerialDebug.printf("JD:      %.6f", Data.Time.JD);  //em dias
      SerialDebug.print("\033[38;1f"); // L36 C1
      SerialDebug.printf("JD2000:  %.6f", Data.Time.JD2000);  //em dias


      SerialDebug.print("\033[2;50f"); // L1 C1
      SerialDebug.printf("Julian Day: %f", Data.Time.JD);

      //SerialDebug.print("\033[3;50f"); // L1 C1
      //SerialDebug.print("J2000(Day): %3.6f", jd_days);

      SerialDebug.print("\033[4;50f"); // L1 C1
      SerialDebug.printf("J2000: %3.6f", Data.Time.JD2000);

      SerialDebug.print("\033[5;50f"); // L1 C1
      SerialDebug.printf("Data.Time.GMST: %3.6f", Data.Time.GMST);

      SerialDebug.print("\033[6;50f"); // L1 C1
      SerialDebug.printf("Data.Time.LST: %3.6f", Data.Time.LST);

      //SerialDebug.print("\033[7;50f"); // L1 C1
      //SerialDebug.printf("LST: %02d:%02d:%02d", int(arHH), abs(int(arMM)), abs(int(arSS)));





      SerialDebug.print("\033[10;1f"); // L1 C1
      SerialDebug.print("Convert AltAz to HaDec:");
      SerialDebug.print("\033[11;1f"); // L1 C1
      SerialDebug.printf("Altitude (Z):  % 4.6f %02dg%02dm%02ds.%03d", Data.Telescope.Altitude.DD, Data.Telescope.Altitude.DMS.D, Data.Telescope.Altitude.DMS.M, Data.Telescope.Altitude.DMS.S, Data.Telescope.Altitude.DMS.SS);
      SerialDebug.print("\033[12;1f"); // L1 C1
      SerialDebug.printf("Azimuth (X):   % 4.6f %02dg%02dm%02ds.%03d", Data.Telescope.Azimuth.DD, Data.Telescope.Azimuth.DMS.D, Data.Telescope.Azimuth.DMS.M, Data.Telescope.Azimuth.DMS.S, Data.Telescope.Azimuth.DMS.SS);
      SerialDebug.print("\033[13;1f"); // L1 C1
      SerialDebug.printf("DEC:           % 4.6f %02dg%02dm%02ds.%03d", Data.Telescope.Declination.DD, Data.Telescope.Declination.DMS.D, Data.Telescope.Declination.DMS.M, Data.Telescope.Declination.DMS.S, Data.Telescope.Declination.DMS.SS);
      SerialDebug.print("\033[14;1f"); // L1 C1
      SerialDebug.printf("RA:            % 4.6f %02dh%02dm%02ds.%03d", Data.Telescope.RightAscension.DT, Data.Telescope.RightAscension.HMS.H, Data.Telescope.RightAscension.HMS.M, Data.Telescope.RightAscension.HMS.S, Data.Telescope.RightAscension.HMS.SS);
      
      SerialDebug.printf("\r\nX: % 3.6f%c (Az) -  % 3.6f%\r\n", Data.Inertial.Euler.X, char(9), Data.Inertial.EulerIIR.X); //Heading
      SerialDebug.printf("Z: % 3.6f%c (Alt) - % 3.6f%\r\n", Data.Inertial.Euler.Z, char(9), Data.Inertial.EulerIIR.Z);
      SerialDebug.printf("Euler Angles Frq: %dHz xget %d milli %d", Data.Inertial.Freq, xTaskGetTickCount(), millis());
      
      //SerialDebug.print("\033[15;1f"); // L1 C1
      //SerialDebug.printf("Az_sin: %f", Az_sin);
      xSemaphoreGive( xSerialDBGSemaphore ); // Now free or "Give" the Serial Port for others.
    }
  }
}