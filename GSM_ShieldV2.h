/* ********************************************************************************************************************************************************************************** */
/* Example of URL: http://145.239.199.14/cgi-bin/econect/receive_data.py?location=aquacosme1&temp_eau=18.73&ph=7.21&int_lumineuse=799.3&oxygene=16.2&conductivite=123.4&redox=-24.71  */

#ifndef GSM_SHIELDV2_H_
#define GSM_SHIELDV2_H_          1

#include      <string.h>
#include      <stdio.h>
#include      <stdint.h>
#include      <stdlib.h>
#include      <GSM.h>

/*************************** Shared compilation directives which have to be activated or inhibited in each header files where they are necessary ***************************/
#define       messagesON                        // also defined in Functions.h

/*************************** CONSTANTS ***************************/
// Global System for Mobile communications and General Packet Radio Service
#define       PinNumberSFR8697        "0000"
#define       Default_AccessPointName "sfr"
#define       SFR_Connect_Web         "websfr"
#define       SFR_Connect_SL          "slsfr"
#define       SFR_Connect_IPNET       "ipnet"
#define       SFR_Connect_InternetPro "internetpro"
#define       SRR_WEB                 "websfr"
#define       SRR_SL                  "slsfr"
#define       SRR_InternetPro         "internetpro"
#define       SRR_IPNET               "ipnet"
#define       SFR_WEBPHONE            "sl2sfr"
#define       SFR_Mobile              "wapsfr"
#define       Orange_Nicolas          "orange"
#define       OfficialSFRapn          "box"
#define       GPRS_Login              ""
#define       GPRS_Password           ""
#define       HeaderHost              "Host: "
#define       StartURL                "http://"
#define       StartFrame              "GET /"
#define       httpGETcde2             "GET / HTTP/1.1"
#define       EndFrame                " HTTP/1.1"
#define       ServerOVH_Address       "145.239.199.14"
#define       CloseFrame              "Connection: close"
#define       ServerPort              80
#define       JLDRnumero              "+33611206107"
#define       EconectReceiveDatas     "/cgi-bin/econect/receive_data.py?location=aquacosme1"
#define       TempDS18B20Topic        "&temp_eau="
#define       pHTopic                 "&ph="
#define       ORPTopic                "&redox="
#define       CondTopic               "&conductivite="
#define       VEML7700Topic           "&int_lumineuse="
#define       DOTopic                 "&oxygene="
#define       Null                    '\0'          // 0 ou '\0'
#define       Sign_Mask               0x80000000
#define       RxPin                   10
#define       TxPin                   3
#define       URL_length              200

/*************************** Flags ***************************/
#define       Flag0                   0

/*************************** compilation directives ***************************/
#define       VerboseAnswer           false

/********************************************** Predefined types **********************************************/
typedef struct GSMProbeMeasures {
  float DO_GSMValue;        // I2C
  float pH_GSMValue;        // I2C
  float ORP_GSMValue;       // I2C
  float EC_GSMValue;        // I2C
  float RTD_GSMValue;       // I2C
  float Temp_GSMValue;      // OneWire
  float Lux_GSMValue;       // I2C
} GSMProbeMeasures_t;

//typedef union GSMflottant {
//  float value;
//  uint8_t byte_value[4];    // little-endian representation
//} MyGSMFloat_t;                // for (k = 4; k > 0; k--) BigEndianFormat[k - 1] = LocalFloat.byte_value[4 - k]; /* big-endian representation */

/* Function prototype or functions interface */
void ReadIPSeverAddress(void);
void DividerGSM_ShiledV2(uint8_t, boolean, char);
boolean initGSMShield(void);
boolean initGSMShield2(void);
void URL_EntirePacket(float, float, float, float, float, float);
//void URL_EntirePacket(void);
void ConvFloatToStringWithSign(float, uint8_t, char *);
uint8_t ConvertUint32ToASCIIChar(char *, uint8_t, uint32_t);
void GSM_Parameters(void);
void SendSMS(String);
uint8_t GetNbrOfCharForGSM(char *);
void software_Reset(void);




#endif                /* GSMSHIELDV2_H_ */



/* END FILE */
