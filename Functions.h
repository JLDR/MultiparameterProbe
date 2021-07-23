/* ******************************************************************************************** */
/* Global functions for the multiparameters probe with Atlas Scientific devices.                */
/* ******************************************************************************************** */
// Author: Jean-Louis Druilhe (jean-louis.druilhe@univ-tlse3.fr)

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_        1

#include      <Wire.h>              // I2C library
#include      <OneWire.h>
#include      <DallasTemperature.h>
#include      <Adafruit_ADS1X15.h>
#include      <avr/interrupt.h>     /* interrupt vectors */
#include      <avr/pgmspace.h>
#include      <math.h>
#include      <string.h>
#include      <stdio.h>
#include      <stdint.h>
#include      <stdlib.h>            /* atof() function */
#include      <avr/io.h>            /* sprintf and printf */
#include      <Arduino.h>
#include      "EEPROM.h"
//#include      <avr/wdt.h>

/*************************** CONSTANTS ***************************/
//#define       LF                  0x0A      // '\n' this acronym is already used by the core
//#define       CR                  0x0D      // '\r' this acronym is already used by the core
#define       Space               0x20
#define       Null                0         // '\0'
#define       DS18B20Temp         10

#define       Timer0_ON           0
#define       Timer1_ON           1
#define       Timer2_ON           2
#define       Timer3_ON           3
#define       Timer4_ON           4
#define       Timer5_ON           5

/*************** Atlas Scientific probes ***************/
#define       Cmd_I               'i'
#define       Cmd_Single_Read     'R'
#define       Cmd_Status          "Status"
#define       Cal_DO_Atmos        "Cal"
#define       Cal_DO_Zero         "Cal,0"
#define       Cal_EC_Dry          "Cal,dry"
#define       Cal_MySolution      "Cal,"
#define       Std_1415us          "Cal,1413"
#define       Std_84us            "Cal,84"
#define       Cmd_Cal_mid         "Cal,mid,"
#define       Cmd_Cal_low         "Cal,low,"
#define       Cmd_Cal_high        "Cal,high,"
#define       Cmd_CalClear        "Cal,clear"
#define       Cmd_NbrPtsCal       "Cal,?"
#define       Cmd_SLEEP           "Sleep"
#define       InventoryText       "Probes inventory"
#define       Chg_add_i2c         "I2C,"
#define       Compensate_temp     "T,"
#define       Request_CompT       "T,?"

/*************************** General Purpose Input Output (GPIO) functions ***************************/
#define       GPIO_pH_isolator    8
#define       GPIO_pH_Switch      9
#define       GPIO_ORP_isolator   24
#define       GPIO_ORP_Switch     26
#define       GPIO_EC_isolator    25
#define       GPIO_EC_Switch      27
#define       GPIO_RTD_isolator   28
#define       GPIO_RTD_Switch     30
#define       GPIO_DO_isolator    29
#define       GPIO_DO_Switch      31
#define       GPIO_DS18_isolator  32
#define       GPIO_DS18_Switch    34
#define       GPIO_VEML_isolator  33
#define       GPIO_VEML_Switch    35

#define       CdeNmosRFM0505      38

/*************** ADS1115 ADC ***************/
#define       valMax              32767
#define       ADCddpmax           4.096
#define       Battery             1     // range between 3 volts and 4.2 volts so gain 1 using power supply +5V and resistive bridge of 1/2 (2 x 47k)
#define       MPPToutput          2     // +5 volts, gain 1 and resistive bridge of 1/2 (2 x 47 KOhms)
#define       SolarPanel          3     // the potential can climb to 18 volts (gain 1 and resistive bridge of 1/11 (20k + 200k) 

/*************** other constants ***************/
#define       zero                0.0001
#define       Sign_Mask           0x80000000                // for float numbers defined with 4 bytes

/*************************** MACRO & Functions ***************************/
//#define wdt_reset() __asm__ __volatile__ ("wdr")            // (Watchdog Reset WDR) https://gcc.gnu.org/onlinedocs/gcc/Extended-Asm.html
//#define __inline__ __attribute__ ((__always_inline__)) void wdt_enable(const uint8_t value)           // library <avr/wdt.h>

/*************************** Flags ***************************/
#define       UsingTimer1Interrupt      0           // used by other modules too
#define       WatchdogDelayArmed        1           // Flag to inform that the watchdog has to be armed
#define       StopTheWatchdogTimer      2
#define       Flag3                     3
#define       Flag4                     4
#define       GSMInitialized            5
#define       APNInitialized            6
#define       Flag7                     7

/*************************** Shared compilation directives which have to be activated or inhibited in each header files where they are necessary ***************************/
#define       messagesON
//#define       ADS115Connected
#define       BusyTimeForProbes         8           // available measure from probe
#define       BusyTimeForDS18B20        3
#define       NbrMinutesToResetGSM      60          // 1 heure
#define       FollowingErrors           2

/********************************************** Predefined types **********************************************/
typedef enum I2CAddresses : uint8_t {
  DissolvedOxygen_Add = 0x61,           // address_DO_EZO
  ORP_Add = 0x62,                       // address_ORP_EZO
  pH_Add = 0x63,                        // address_pH_EZO
  Conductivity_Add = 0x64,              // address_EC_EZO
  RTD_Add = 0x66,                       // address_RTD_EZO
  VEML7700_Add = 0x10,
  NoI2C_Add = 0x00
} I2CAddresses_t;

typedef struct I2CStampsConnected {     // all I2C sensors
  boolean DO_Probe = false;
  boolean ORP_Probe = false;
  boolean pH_Probe = false;
  boolean EC_Probe = false;
  boolean RTD_Probe = false;
  boolean VEML7700_Probe = false;
} I2CProbesConnected_t;

typedef struct SamplingDelay {
  boolean RepeatedMeasures;
  uint16_t NbrSeconds;
  uint16_t NbrMinutes;
  uint32_t CompleteIntervalInSeconds;
} SamplingDelay_t;

typedef enum token : uint8_t {        // ph, oxy, orp, cond, rtd
  NoToken = 0x00,                     // only for Atlas Scientific probes for calibration with same control commands
  oxy = 0x01,
  orp = 0x02,
  cond = 0x03,
  rtd = 0x04,
  ph = 0x05
} token_t;

typedef union flottant {
  float value;
  uint8_t byte_value[4];    // little-endian representation
} MyFloat_t;                // for (k = 4; k > 0; k--) BigEndianFormat[k - 1] = LocalFloat.byte_value[4 - k]; /* big-endian representation */

typedef struct ProbeMeasures {
  float DO_FloatValue;      // I2C
  float pH_FloatValue;      // I2C
  float ORP_FloatValue;     // I2C
  float EC_FloatValue;      // I2C
  float RTD_FloatValue;     // I2C
  float Temp_FloatValue;    // OneWire
  float Lux_FloatValue;     // I2C
} ProbeMeasures_t;

typedef struct ElapsedTime {
  uint16_t minutes = 0;
  uint8_t seconds = 0;
  boolean ResetGSM = false;
} ElapsedTime_t;

typedef struct Voltages {
  float ddp_bat = 0.0;
  float ddp_mppt = 0.0;
  float ddp_panel = 0.0;
} Voltages_t;

typedef enum DeviceInProgress : uint8_t {     // to select only one device
  pHProbe = 0,
  ORPProbe,
  ECProbe,
  RTDProbe,
  DOProbe,
  VEML7700,
  DS18B20,
  AllProbes                 // For this state, all probes will be polling and the power applied for each device will depend of their boolean individual switch
} DeviceInProgress_t;

typedef struct PowerSwitches {
  boolean pHProbePowered = false;
  boolean ORPProbePowered = false;
  boolean ECProbePowered = false;
  boolean RTDProbePowered = false;
  boolean DOProbePowered = false;
  boolean VEML7700Powered = false;
  boolean DS18B20Powered = false;
  DeviceInProgress_t TheSelectedDevice;
} PowerSwitches_t;

/* Function prototype or functions interface */
void Init_Timers(uint8_t, uint8_t, uint16_t, uint8_t, uint16_t, uint16_t, uint16_t);
I2CProbesConnected_t scani2c(void);
uint8_t check_i2c_connection(uint8_t);
void I2C_call(char *, I2CAddresses_t, uint8_t);
void DisplayFrameFromStamp(char *);
char *parseInfo(I2CAddresses_t);
void help(String);
uint16_t detect_entier(char *, String);
boolean detect_float(char *);
uint16_t Convert_DecASCII_to_uint16(char *);
uint32_t Convert_DecASCII_to_uint32(char *);
uint8_t ConvertUint16ToASCIIChar(char *, uint16_t);
uint8_t ConvertUint32ToASCIIChar(char *, uint32_t);
float AtlasProbesMeasure(I2CAddresses_t);
void change_add_I2C(String);
void CheckSetFocus(String);
void ShowFocus(void);
void CompensatedTemp_pH_DO(String);
void Calibration(String);
void DeleteCalibration(String);
void DisplayNbrCalPoints(String);
void initI2C_Devices(void);
boolean DallasTemperatureSearch(void);
void AfficheAdresseCapteur(void);
void Divider(uint8_t, boolean, char);
float MeasureTemp(void);
void ConvFloatToString(float, uint8_t, char *);
void FillMyAnswerArray(void);
uint16_t ConvASCIItoUint16(char *);
SamplingDelay_t SamplingDelayMeasure(String, SamplingDelay_t);
ProbeMeasures_t Reading_probes(String);
ProbeMeasures_t RandomValues(void);
float orpMeasure(boolean);
float ConductivityMeasure(boolean);
float pHMeasure(boolean);
float OxyMeasure(boolean);
float TempMeasure(boolean);
uint8_t GetNbrOfChar(char *);
uint8_t DisplayAsciiArray(char *);
uint8_t Concatenate2Arrays(char *, char *, char *);
void InventoryProbes(I2CProbesConnected_t, boolean);
void SleepMode(I2CAddresses_t);
ElapsedTime_t IncrementMyGSMtime(ElapsedTime_t);
void DisplayArrayContentFunctions(char *, boolean);
void ADCStart(adsGain_t);
Voltages_t AcquireVoltageValues(void);
void GPIOConfigurationAndPowerON(void);
void IsolatorAndPowerOFF(void);
void PowerSupplyForDevices(PowerSwitches_t);
  


#endif /* FONCTIONS_H_ */
