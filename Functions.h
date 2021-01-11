/* ******************************************************************************************** */
/* Ensemble de fonctions dédiées pour cette application.                    */
/* ******************************************************************************************** */
#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_        1

#include      <Wire.h>              // I2C library
#include      <OneWire.h>
#include      <DallasTemperature.h>
#include      <avr/interrupt.h>     /* interrupt vectors */
#include      <avr/pgmspace.h>
#include      <math.h>
#include      <string.h>
#include      <stdio.h>
#include      <stdint.h>
#include      <stdlib.h>            /* atof() function */
#include      <avr/io.h>            /* sprintf and printf */
#include      <Arduino.h> 

/*************************** CONSTANTS ***************************/
#define       LF                  0x0A      // '\n'
#define       CR                  0x0D      // '\r'
#define       Space               0x20
#define       Null                0         // '\0'
#define       DS18B20Temp         10
#define       Timer0_ON           0
#define       Timer1_ON           1
#define       Timer2_ON           2
#define       Timer3_ON           3
#define       Timer4_ON           4
#define       Timer5_ON           5

#define       Cmd_I               'I'
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

#define       Chg_add_i2c         "I2C,"
#define       Compensate_temp     "T,"
#define       Request_CompT       "T,?"

#define       zero                0.000001
#define       Sign_Mask           0x80000000        // for float numbers defined with 4 bytes

#define       messagesON
//#define       SerialPlotting


/********************************************** Predefined types **********************************************/
typedef enum I2CAddresses : uint8_t {
  DissolvedOxygen_Add = 0x61,           // address_DO_EZO
  ORP_Add = 0x62,                       // address_ORP_EZO
  pH_Add = 0x63,                        // address_pH_EZO
  Conductivity_Add = 0x64,              // address_EC_EZO
  RTD_Add = 0x66,                       // address_RTD_EZO
  VEML7700_Add = 0x10,
  NoI2C_Add = 0x00
} Atlas_address_t;

typedef struct I2CStampsConnected {
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
} SamplingDelay_t;

typedef enum token : uint8_t {        // ph, oxy, orp, cond, rtd
  NoToken = 0x00,                     // only for Atlas Scientific probes
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
  float DO_FloatValue;
  float pH_FloatValue;
  float ORP_FloatValue;
  float EC_FloatValue;
  float Temp_FloatValue;
  float Lux_FloatValue;
} ProbeMeasures_t;

/* prototypage des fonctions de la bibliothèque */
void Init_Timers(uint8_t, uint8_t, uint16_t, uint8_t, uint16_t, uint16_t, uint16_t);
I2CProbesConnected_t scani2c(void);
uint8_t check_i2c_connection(uint8_t);
void I2C_call(char *, Atlas_address_t, uint8_t);
void DisplayFrameFromStamp(char *);
char *parseInfo(Atlas_address_t);
void help(void);
uint16_t detect_entier(char *, String);
float detect_float(char *, String);
uint16_t Convert_DecASCII_to_uint16(char *);
uint8_t ConvertUint16ToASCIIChar(char *, uint16_t);
uint8_t ConvertUint32ToASCIIChar(char *, uint32_t);
float AtlasProbesMeasure(Atlas_address_t);
void change_add_I2C(String);
void CheckSetFocus(String);
void ShowFocus(void);
void CompensatedTemp_pH_DO(String);
void Calibration(String);
void DeleteCalibration(String);
void DisplayNbrCalPoints(void);
void initI2C_Devices(void);
boolean DallasTemperatureSearch(void);
void AfficheAdresseCapteur(void);
void Divider(uint8_t, char);
float MeasureTemp(void);
void ConvFloatToString(float, uint8_t, char *);
void FillMyAnswerArray(void);
uint16_t ConvASCIItoUint16(char *);
SamplingDelay_t SamplingDelayMeasure(String, SamplingDelay_t);
ProbeMeasures_t Reading_probes(String);
float orpMeasure(boolean);
float ConductivityMeasure(boolean);
float pHMeasure(boolean);
float OxyMeasure(boolean);
float TempMeasure(boolean);







#endif /* FONCTIONS_H_ */
