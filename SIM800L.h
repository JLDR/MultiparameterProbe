/* ********************************************************************************************************************************************************************************** */
/* Example of URL: http://145.239.199.14/cgi-bin/econect/receive_data.py?location=aquacosme1&temp_eau=18.73&ph=7.21&int_lumineuse=799.3&oxygene=16.2&conductivite=123.4&redox=-24.71  */

#ifndef SIM800L_H_
#define SIM800L_H_          1

#include      <string.h>
#include      <stdio.h>
#include      <stdint.h>
#include      <stdlib.h>
#include      <GSMSimHTTP.h>        // => #include "GSMSimGPRS.h" => #include "GSMSim.h" => #include <Arduino.h>
#include      <GSMSimSMS.h>

/*************************** Shared compilation directives which have to be activated or inhibited in each header files where they are necessary ***************************/
#define       TextOnTerminalForSIM800L          // has to be unique or specific to this module (which is 2 files .cpp and .h)
//#define       SIM800Lpresent                    // used to initialize the GSM board
#define       GSMChangeSIMcodeFromPUKcode       // used to change the SIM code knowing the Personal Unblocking Key code

/*************************** CONSTANTS ***************************/
// Global System for Mobile communications and General Packet Radio Service
#define       PinNumberInUse              "8697"
#define       PUKcodeSFR8697              "64701796"
#define       Default_AccessPointName     "sfr"
#define       SFR_Connect_Web             "websfr"          // => SMS OK
#define       SFR_Connect_SL              "slsfr"
#define       SFR_Connect_IPNET           "ipnet"
#define       SFR_Connect_InternetPro     "internetpro"
#define       SRR_WEB                     "websfr"
#define       SRR_SL                      "slsfr"
#define       SRR_InternetPro             "internetpro"
#define       SRR_IPNET                   "ipnet"
#define       SFR_WEBPHONE                "sl2sfr"
#define       SFR_Mobile                  "wapsfr"
#define       Orange_Nicolas              "orange"
#define       OfficialSFRapn              "box"
#define       GPRS_Login                  ""
#define       GPRS_Password               ""
#define       HeaderHost                  "Host: "
#define       StartURL                    "http://"
#define       StartFrame                  "GET /"
#define       httpGETcde2                 "GET / HTTP/1.1"
#define       EndFrame                    " HTTP/1.1"
#define       ServerOVH_Address           "145.239.199.14"
#define       ServerArnaud                "82.64.109.143"
#define       CloseFrame                  "Connection: close"
#define       ServerPort                  80
#define       JLDRnumero                  "+33611206107"
#define       EconectReceiveDatas         "cgi-bin/econect/receive_data.py?location=aquacosme1"
#define       EconectAquacosme2Site       "cgi-bin/econect/prepare_data.py?location=aquacosme2"
#define       TempDS18B20Topic            "&temp_eau="
#define       pHTopic                     "&ph="
#define       ORPTopic                    "&redox="
#define       CondTopic                   "&conductivite="
#define       VEML7700Topic               "&int_lumineuse="
#define       DOTopic                     "&oxygene="
#define       BatteryTopic                "&ddp_bat="
#define       MpptTopic                   "&ddp_mppt="
#define       SolarTopic                  "&ddp_pan="
#define       Null                        '\0'          // 0 ou '\0'
#define       Sign_Mask                   0x80000000
#define       RxPin                       10
#define       TxPin                       3
#define       URL_length                  250

/********************************** GSM commands ***********************************************/
#define       SMSintoTextMode             "AT+CMGF=1"
#define       TestPinCode                 "AT+CPIN=?"
#define       GetPinCode                  "AT+CPIN?"
#define       SignalQuality               "AT+CSQ"
#define       GetIMEInumber               "AT+CGSN"
#define       TestCommand                 "AT\r\n"
#define       SetBaudRate                 "AT+IPR="
#define       Identification              "AT+GSV"
#define       NetworkRegistration         "AT+CREG=?"             // +CREG: (list of supported <n>s) => example: <AT+CREG?> response: <+CREG: 0,2 OK> (Call Ready SMS Ready)
#define       SetBaudRate                 "AT+IPR="
#define       SignalQualityReport         "AT+CSQ=?"
#define       AntennaDetecting            "AT+CANT=?"
#define       RquestCompTACapList         "AT+GCAP"               // TA: Timing Advance
#define       RquestManufacIdent          "AT+GMI=?"
#define       RquestTAModelIdent          "AT+GMM=?"
#define       RquestTARevisionIdent       "AT+GMR=?"
#define       RquestGlobalObject          "AT+GOI=?"
#define       InitHTTPService             "AT+HTTPINIT"
#define       StopHTTPService             "AT+HTTPTERM"
#define       SetHTTPParamValues          "AT+HTTPPARA"
#define       InputHTTPData               "AT+HTTPDATA"
#define       HTTPMethodAction            "AT+HTTPACTION"
#define       ReadHTTPServResp            "AT+HTTPREAD"
#define       ReadHTTPStatus              "AT+HTTPSTATUS"
#define       ReadHTTPHeaderInf           "AT+HTTPHEAD"
#define       MEtoMinimumFunction         "AT+CFUN="              // ME: Mobile Equipment
#define       FacilityLock                "AT+CLCK="

/*************************** General Purpose Input Output (GPIO) functions ***************************/
#define       ResetPIN                    12

/*************************** Flags ***************************/
#define       UsingTimer1Interrupt    0
#define       WatchdogDelayArmed      1         // Flag to inform that the watchdog has to be armed
#define       StopTheWatchdogTimer    2
#define       Flag3                   3
#define       Flag4                   4
#define       GSMInitialized          5
#define       APNInitialized          6
#define       Flag7                   7

/*************************** compilation directives which are used only by this module ***************************/
#define       BusyTimeForGSM          45        // maximum time for GSM initialization in seconds
#define       BusyTimeForGPRSconnect  20        // GPRS initialization
#define       BusyTimeForClient       5

/********************************************** Predefined types **********************************************/
typedef struct GSMProbeMeasures {
  float DO_GSMValue;        // I2C
  float pH_GSMValue;        // I2C
  float ORP_GSMValue;       // I2C
  float EC_GSMValue;        // I2C
  float RTD_GSMValue;       // I2C
  float Temp_GSMValue;      // OneWire
  float Lux_GSMValue;       // I2C
  float ddp_bat;            // I2C
  float ddp_mppt;           // I2C
  float ddp_pan;            // I2C
} GSMProbeMeasures_t;

//typedef enum PinState : int {
//  Ready = 0,
//  SIM_PIN = 1,
//  SIM_PUK = -1,
//  Error = -2
//} PinState_t;

/* Function prototype or functions interface */
void ReadIPSeverAddress(void);
void DividerSIM800L(uint8_t, boolean, char);
void InitSIM800L(void);
boolean URL_EntirePacket(float, float, float, float, float, float, float, float, float);
void ConvFloatToStringWithSign(float, uint8_t, char *);
uint8_t ConvertUint32ToASCIIChar(char *, uint8_t, uint32_t);
void GSM_Parameters(void);
void SendSMS(String);
uint8_t GetNbrOfCharForGSM(char *);
void software_Reset(void);
void DisplayArrayContent(char *, boolean);
void FillShortAnswerArray(void);
void TestPINmode(void);

#endif                /* SIM800L_H_ */



/* END FILE */
