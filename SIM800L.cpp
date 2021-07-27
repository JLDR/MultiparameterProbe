// Author: Jean-Louis Druilhe (jean-louis.druilhe@univ-tlse3.fr)
// NOTE: 
//
// Global System for Mobile communications SIM800L
// General Packet Radio Service
// https://sites.google.com/site/lteencyclopedia/lte-acronyms
// About acronym TA: https://www.telecomhall.net/t/parameter-timing-advance-ta/6390 Timing Advance
// About acronym MT: Mobile termination
// About acronym ME: Mobile equipment

#include      "SIM800L.h"

/******************************* Local variables used by this module *******************************/
char                  LocalCharArray[32];
char                  URL_EconectServer[URL_length];
GSMProbeMeasures_t    MyGSMMeasures;                  // useful if this variable is a common shared vlaue by this module
char                  FloatToAsciiArray[10];
char                  PinNumberArray[5];
char                  AccessPointNameArray[12];
char                  PinCodeOfTheSimCard[5];
char                  PUKcodeofTheSimCard[9];
char                  Answer[12];

/******************************* External variables (Solution to share state from other modules) *******************************/
extern volatile uint16_t    cmpt1, cmpt2, cmpt3, cmpt4, cmpt5, cmpt_5ms;                  // Timers (also defined in Functions.h)
extern volatile uint8_t     compt1, compt2, compt3, compt4, compt5, cmpt_100us;           // Timers (also defined in Functions.h)
extern uint8_t              BusyTimeForWatchdog, Flags;                                   // global variables defined in Functions.h and tied to the timers and the watchdog

/* Class instances which are objects with public methods and private attributes */
// GSMSim Library (https://github.com/erdemarslan/GSMSim) ou (https://www.arduinolibraries.info/libraries/gsm-sim)
GSMSimHTTP http(Serial1, ResetPIN);                 // GSMSimHTTP(Stream& s, unsigned int resetPin) : GSMSimGPRS(s, resetPin) {...}
//GSMSimSMS MySMS(Serial1, ResetPIN);                 // GSMSimSMS(Stream& s, unsigned int resetPin) : GSMSim(s, resetPin) {...}
/**********************************************************************************************************************************/
/* Function to read the IP address of the server. The IP address is loaded using a constant declaration as #define                */
/**********************************************************************************************************************************/
void ReadIPSeverAddress() {
  String LocalCde;
  LocalCde.reserve(20);
  LocalCde = ServerOVH_Address;             // #define ServerOVH_Address "145.239.199.14"
  memset(LocalCharArray, Null, sizeof(LocalCharArray));
  LocalCde.toCharArray(LocalCharArray, LocalCde.length() + 1);
  DividerSIM800L(80, true, '-');
  Serial.print(F("IP address is: "));
  DisplayArrayContent(&LocalCharArray[0], true);
  DividerSIM800L(80, true, '-');
}
/****************************************************************************************************/
/* Local divider (GSM_ShiledV2)                                                                     */
/****************************************************************************************************/
void DividerSIM800L(uint8_t nbr_carac, boolean CRLFChar, char caract) {
  uint8_t i;
  for (i = 0; i < nbr_carac; i++) Serial.print(caract);
  if (CRLFChar == true) Serial.println();
}
/****************************************************************************************************/
/* Function to initialize the GSM SIM800L device connected to the UART1 of the MEGA card.           */
/* enum GSM3_NetworkStatus_t { ERROR, IDLE, CONNECTING, GSM_READY, GPRS_READY,                      */
/* TRANSPARENT_CONNECTED, OFF}; (GSM3MobileAccessProvider.h)                                        */
/****************************************************************************************************/
void InitSIM800L() {
  String AccesPointName;
  String PinNumber;
  String IP_address;
  boolean Status;
  uint16_t SignQuality;;
  AccesPointName.reserve(12);
  PinNumber.reserve(5);
  IP_address.reserve(20);
  AccesPointName = OfficialSFRapn;                        // "box"
  PinNumber = PinNumberInUse;                             // The Pin code in use
  
  DividerSIM800L(80, true, '-');
  Serial.println(F("*** Starting the web client services ***"));
  memset(AccessPointNameArray, Null, sizeof(AccessPointNameArray));
  AccesPointName.toCharArray(AccessPointNameArray, AccesPointName.length() + 1);
  memset(PinNumberArray, Null, sizeof(PinNumberArray));
  PinNumber.toCharArray(PinNumberArray, PinNumber.length() + 1);
  Flags &= ~((1<<APNInitialized)|(1<<GSMInitialized));            // from initiation during setup, flags have been cleared
  Serial.print(F("[SIM800L] Access Point Name used: "));
  DisplayArrayContent(&AccessPointNameArray[0], true);
  Serial.print(F("[SIM800L] Pin number: "));
  DisplayArrayContent(&PinNumberArray[0], true);

  Serial.println("[SIM800L] Connecting to GSM network");
  Serial.print(F("[SIM800L] GSM Initialization pending"));
  compt1 = 0;                                                     // criterio to display '.'
  Flags |= ((1<<UsingTimer1Interrupt)|(1<<WatchdogDelayArmed));   // for a mandatory interruption
  BusyTimeForWatchdog = BusyTimeForGSM;
  http.init();                                                    // inheritance from GSMSimGPRS.h which inherits from GSMSim.h
  Flags |= (1<<StopTheWatchdogTimer);
  Flags &= ~(1<<UsingTimer1Interrupt);
  Flags |= (1<<GSMInitialized);
  Serial.println(F("\n[SIM800L] GSM initialized"));
  TestPINmode();
  Serial.print("[SIM800L] Is Module Registered to Network? ");
  Status = http.isRegistered();                                   // bool GSMSim::isRegistered() {...} "AT+CREG?"
  if (Status == true) Serial.println(F("=> Registered"));
  else Serial.println(F("=> Not registerd or registration denied"));
  Serial.print("[SIM800L] Signal Quality: ");
  SignQuality = http.signalQuality();                             // unsigned int GSMSim::signalQuality() {...}
  Serial.println(SignQuality, DEC);
  //Serial.print("[SIM800L] Service Provider Name from SIM: ");
  //Serial.println(http.operatorNameFromSim());                     // "AT+CSPN=?" String GSMSim::operatorNameFromSim() {...} (blocking function)

  Serial.println(F("[SIM800L] GPRS Access Point Name defined"));
  http.gprsInit(AccesPointName);                                  // non blocking function (void GSMSimGPRS::gprsInit(String apn) {APN = apn;})
  Flags |= (1<<APNInitialized);
  Serial.print("[SIM800L] Get IP address: ");
  Serial.println(http.getIP());
  //IP_address = http.getIP();                                    // blocking function
  //Serial.println(IP_address);
  Serial.print("Is module connected to GPRS? ");
  Status = http.connect();
  if (Status == true) Serial.println(F("GPRS Connection is activated"));
  else Serial.println(F("GPRS Connection is out of order"));
  
  DividerSIM800L(80, true, '-');
}
/**********************************************************************************************************************************/
/* Function to gather all text topics including IP address to make an URL for GPRS. The string which contains all formated datas  */
/* is LocalURL who is a local variable for this function.                                                                         */
/* frame received: URL_EntirePacket(AllMeas.DO_FloatValue, AllMeas.pH_FloatValue, AllMeas.ORP_FloatValue, AllMeas.EC_FloatValue,  */
/* AllMeas.Temp_FloatValue, AllMeas.Lux_FloatValue);                                                                              */
/* http://145.239.199.14/cgi-bin/econect/receive_data.py?location=aquacosme1&temp_eau=18.73&ph=7.21&int_lumineuse=799.3&oxygene=16.2&conductivite=123.4&redox=-24.71
/**********************************************************************************************************************************/
boolean URL_EntirePacket(float DO, float pH, float ORP, float EC, float DS18, float Lux, float ddpBAT, float ddpMPPT, float ddpSUN) {
  //GSMProbeMeasures_t MyGSMMeasures;           // useful if this variable is a common shared value by this module
  
  MyGSMMeasures.DO_GSMValue = DO;
  MyGSMMeasures.pH_GSMValue = pH;
  MyGSMMeasures.ORP_GSMValue = ORP;
  MyGSMMeasures.EC_GSMValue = EC;
  MyGSMMeasures.Temp_GSMValue = DS18;
  MyGSMMeasures.Lux_GSMValue = Lux;
  MyGSMMeasures.ddp_bat = ddpBAT;
  MyGSMMeasures.ddp_mppt = ddpMPPT;
  MyGSMMeasures.ddp_pan = ddpSUN;

  boolean Status;
  boolean ConnectError = false;                 // if GPRS is out of order
  char MyChar;
  String LocalURL;
  String AsciiFloat;
  String OVH_IP;
  String AnswerFromSIM800L;
  char *Lcl_ptr;
  uint8_t NbrChar;
  uint8_t ReadFlags;
  LocalURL.reserve(URL_length);                 // #define URL_length 250
  AsciiFloat.reserve(10);
  OVH_IP.reserve(15);
  AnswerFromSIM800L.reserve(50);

  Serial1.print("AT");

  LocalURL.concat(GET_Command);                  // "GET /"
  //LocalURL.concat(EconectReceiveDatas);         // "cgi-bin/econect/receive_data.py?location=aquacosme1"
  LocalURL.concat(EconectSiteFrancon);          // "cgi-bin/econect/receive_data.py?location=francon"
  LocalURL.concat(TempDS18B20Topic);            // "&temp_eau="
  //MyGSMMeasures.Temp_GSMValue = 18.563;
  ConvFloatToStringWithSign(MyGSMMeasures.Temp_GSMValue, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
  LocalURL.concat(pHTopic);                     // "&ph="
  //MyGSMMeasures.pH_GSMValue = 8.651;
  ConvFloatToStringWithSign(MyGSMMeasures.pH_GSMValue, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
  LocalURL.concat(VEML7700Topic);               // "&int_lumineuse="
  //MyGSMMeasures.Lux_GSMValue = 12764.943;
  ConvFloatToStringWithSign(MyGSMMeasures.Lux_GSMValue, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
  LocalURL.concat(DOTopic);                     // "&oxygene="
  //MyGSMMeasures.DO_GSMValue = 7.543;
  ConvFloatToStringWithSign(MyGSMMeasures.DO_GSMValue, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
  LocalURL.concat(CondTopic);                   // "&conductivite="
  //MyGSMMeasures.EC_GSMValue = 329.538;
  ConvFloatToStringWithSign(MyGSMMeasures.EC_GSMValue, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
  LocalURL.concat(ORPTopic);                    // "&redox="
  //MyGSMMeasures.ORP_GSMValue = 216.654;
  ConvFloatToStringWithSign(MyGSMMeasures.ORP_GSMValue, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
  LocalURL.concat(BatteryTopic);                // "&ddp_bat="
  //MyGSMMeasures.ddp_bat = 4.12345;
  ConvFloatToStringWithSign(MyGSMMeasures.ddp_bat, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
//  LocalURL.concat(MpptTopic);                   // "&ddp_mppt="
//  //MyGSMMeasures.ddp_mppt = 5.0461;
//  ConvFloatToStringWithSign(MyGSMMeasures.ddp_mppt, 2, FloatToAsciiArray);
//  Lcl_ptr = &FloatToAsciiArray[0];
//  AsciiFloat = String(Lcl_ptr);
//  LocalURL.concat(AsciiFloat);
  LocalURL.concat(SolarTopic);                  // "ddp_pan="
  //MyGSMMeasures.ddp_pan = 16.54378;
  ConvFloatToStringWithSign(MyGSMMeasures.ddp_pan, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
                    
  LocalURL.concat(EndFrame);                    // #define EndFrame " HTTP/1.1"
  LocalURL.concat("\r\n");
  LocalURL.concat(HeaderHost);                  // "Host: "
  LocalURL.concat(ServerOVH_Address);           // "145.239.199.14"
  //LocalURL.concat(ServerArnaud);                // "82.64.109.143"
  LocalURL.concat("\r\n");
  LocalURL.concat(CloseFrame);                  // #define CloseFrame "Connection: close"
  LocalURL.concat("\r\n\r\n");

  memset(URL_EconectServer, Null, sizeof(URL_EconectServer));           // global array URL_EconectServer[URL_length];
  LocalURL.toCharArray(URL_EconectServer, LocalURL.length() + 1);
  NbrChar = GetNbrOfCharForGSM(&URL_EconectServer[0]);
  Lcl_ptr = &URL_EconectServer[0];
  DividerSIM800L(80, true, '-');
  
  #ifdef TextOnTerminalForSIM800L
    Serial.print(F("URL for server or Json frame: "));
    DisplayArrayContent(&URL_EconectServer[0], true);
    Serial.print(F("Length of the frame in number of characters: "));
    Serial.println(NbrChar, DEC);
  #endif
  
  #ifdef SIM800Lpresent
    ReadFlags = Flags;
    ReadFlags &= (1<<APNInitialized);
    if (ReadFlags == 0) InitSIM800L();                              // GSM intialization after cut off connection in program
    Serial.print(F("[SIM800L] GPRS connection pending"));
    compt1 = 0;                                                     // criterio to display '.'
    Flags |= ((1<<UsingTimer1Interrupt)|(1<<WatchdogDelayArmed));   // flag WatchdogDelayForGPRS allows to activate
    BusyTimeForWatchdog = BusyTimeForGPRSconnect;
    Status = http.connect();                                   // blocking function
    Flags |= (1<<StopTheWatchdogTimer);                             // Timer3 will reset the watchdog
    Flags &= ~(1<<UsingTimer1Interrupt);
    if (Status == true) {
      Serial.println(F("[SIM800L] Sending data in progress"));
      AnswerFromSIM800L = http.get(LocalURL, true);            // String GSMSimHTTP::get(String url, bool read) {...}
      Serial.print(F("[SIM800L] Answer from SIM800L: "));
      Serial.println(AnswerFromSIM800L);
      ConnectError = false;                                         // local variable returned to inform the calling module 
    } else {
      Serial.println(F("[SIM800L] HTTP command is invalid"));
      ConnectError = true;
    }
    Flags &= ~(1<<APNInitialized);                                  // in order to restart a new initialization of the GSM for each GPRS frame
    Status = http.closeConn();
    Serial.print(F("Network Status from GPRS after switching off: "));
    if (Status == true) Serial.println(F("OK"));
    else Serial.print(F("ERROR"));
  #endif
  
  DividerSIM800L(80, true, '-');
  return ConnectError;
}
/********************************************************************************************************/
/* Function to replace the equivalent method dtostrf.                                                   */
/* char *dtostrf(double val, signed char width, unsigned char prec, char *s)                            */
/* The aim is to fill the char array identified with the litteral values of a double or float number.   */
/* signed char width : number of alphanumeric values, comma included ('.').                             */
/* unsigned char prec : precision of the float or number of digits just after the comma.                */
/* essential link to get an example for conversion: https://www.esp8266.com/viewtopic.php?t=3592        */
/********************************************************************************************************/
void ConvFloatToStringWithSign(float ConvertFloat, uint8_t NbrDecimals, char *DestArray) {
  uint8_t k;
  uint32_t IntegerResult;
  char Scratch_tab[10];       // to convert uint32_t in ASCII format
  uint8_t NbrChar;
  uint8_t CommaPosition;

  if (ConvertFloat < 0) {
    *(DestArray++) = '-';
    ConvertFloat *= -1.0;
  }
  
  switch (NbrDecimals) {
    case 0:
      //IntegerResult = (uint32_t)LocalFloat.value;
      IntegerResult = (uint32_t)ConvertFloat;
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], sizeof(Scratch_tab), IntegerResult);
      for (k = 0; k < NbrChar; k++) *(DestArray++) = Scratch_tab[k];
      *DestArray = Null;
      break;
    case 1:
      //IntegerResult = (uint32_t)(LocalFloat.value * 10.0);                      // 23.8421 x 10 = 238.421 => 238
      IntegerResult = (uint32_t)(ConvertFloat * 10.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], sizeof(Scratch_tab), IntegerResult);       // NbrChar = 3
      CommaPosition = NbrChar - 1;                                              // CommaPosition = 2
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      *(DestArray++) = Scratch_tab[CommaPosition];
      *DestArray = Null;
      break;
    case 2:
      //IntegerResult = (uint32_t)(LocalFloat.value * 100.0);
      IntegerResult = (uint32_t)(ConvertFloat * 100.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], sizeof(Scratch_tab), IntegerResult);
      CommaPosition = NbrChar - 2;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 2; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;
    case 3:
      //IntegerResult = (uint32_t)(LocalFloat.value * 1000.0);
      IntegerResult = (uint32_t)(ConvertFloat * 1000.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], sizeof(Scratch_tab), IntegerResult);
      CommaPosition = NbrChar - 3;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 3; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;
    case 4:
      //IntegerResult = (uint32_t)(LocalFloat.value * 10000.0);
      IntegerResult = (uint32_t)(ConvertFloat * 10000.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], sizeof(Scratch_tab), IntegerResult);
      CommaPosition = NbrChar - 4;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 4; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;
    case 5:
      //IntegerResult = (uint32_t)(LocalFloat.value * 100000.0);
      IntegerResult = (uint32_t)(ConvertFloat * 100000.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], sizeof(Scratch_tab), IntegerResult);
      CommaPosition = NbrChar - 5;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 5; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;      
    default:                                        // only TWO decimals
      //IntegerResult = (uint32_t)(LocalFloat.value * 100.0);
      IntegerResult = (uint32_t)(ConvertFloat * 100.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], sizeof(Scratch_tab), IntegerResult);
      CommaPosition = NbrChar - 2;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 2; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;   
  }
}
/**********************************************************************************************************************************/
/* Function to convert an uint32_t (hexadecimal form) into a decimal ASCII representation to replace sprintf for long integer.    */
/* The conversion is made from low significant bit to high significant bit and the number of significant digit is returned.       */
/* The array in which the ascii conversion is deposited begin with the zero index for the most significant digit.                 */
/* This function return the number of significative digits content in the array.                                                  */
/**********************************************************************************************************************************/
uint8_t ConvertUint32ToASCIIChar(char *ptrTAB, uint8_t ArraySize, uint32_t valToConvert) {   // 10 possible characters from 0 to 4,294,967,295
  char *ptrINIT;
  uint8_t k;
  uint8_t indice;
  uint8_t result_modulo;
  uint32_t result_division;
  ptrINIT = ptrTAB;
  for (k = 0; k < ArraySize; k++) *(ptrTAB++) = Null;   // to initialize the array ConvUintxx_tToAsciiChar[]
  indice = 9;                                           // the low significant digit in the char array (unity)
  ptrTAB = ptrINIT;
  ptrTAB += indice * sizeof(char);                      // to fix the low digit
  do {
    result_modulo = (uint8_t)(valToConvert % 0x0A);
    *(ptrTAB--) = (char)(result_modulo + 0x30);         // ASCII char to display
    indice--;
    result_division = (valToConvert - result_modulo) / 0x0A;
    valToConvert = result_division;                     // new value for which we have to define the modulo
  } while (result_division > 9);                        // if result is between 0 and 15, we can identify all characters to be displayed
  *ptrTAB = (char)(0x30 + result_division);
  ptrTAB = ptrINIT;                                     // first position in the array
  ptrINIT += indice * sizeof(char);                     // to retrieve the last position of the most significant digit
  for (k = 0; k < 10 - indice; k++) *(ptrTAB++) = *(ptrINIT++);   // to retrieve an array starting with the fisrt position [0]
  *ptrTAB = Null;
  return 10 - indice;
}
/**********************************************************************************************************************************/
/* Function to read parameters from the GSM modem.                                                                                */
/* Trying to retrieve the International Mobile Equipment Identify (IMEI)                                                          */
/**********************************************************************************************************************************/
void GSM_Parameters() {
  String GSM_Answer;
  uint8_t ReadFlags;
  GSM_Answer.reserve(50);

  ReadFlags = Flags;
  ReadFlags &= (1<<GSMInitialized);
  if (ReadFlags != 0) {
    GSM_Answer = http.moduleIMEI();
    GSM_Answer.replace("\n", "");
    if (GSM_Answer != "") {
      Serial.print(F("IMEI code: "));
      Serial.println(GSM_Answer);
    }
    GSM_Answer = http.operatorName();
    Serial.print(F("Operator name: "));
    Serial.println(GSM_Answer);
    GSM_Answer = http.moduleManufacturer();
    Serial.print(F("Module Manufacturer: "));
    Serial.println(GSM_Answer);
    GSM_Answer = http.moduleModel();
    Serial.print(F("Module model: "));
    Serial.println(GSM_Answer);
    GSM_Answer = http.moduleRevision();
    Serial.print(F("Module Revision: "));
    Serial.println(GSM_Answer);
    GSM_Answer = http.moduleIMSI();
    Serial.print(F("International Mobile Subscriber Identity: "));
    Serial.println(GSM_Answer);
    GSM_Answer = http.moduleICCID();
    Serial.print(F("Integrated Circuit Card Identifier: "));
    Serial.println(GSM_Answer);
  } else Serial.println(F("GSM has not been initiated..."));
}
/**********************************************************************************************************************************/
/* Function to send a SMS message to a mobile phone.                                                                              */
/* The control command looks like 'sms_'<My text>                                                                                 */
/**********************************************************************************************************************************/
//void SendSMS(String Cde_received) {
//  boolean Status;
//  uint8_t ReadFlags;
//  String Answer;
//  String PhoneNumber;
//  char Message[50];
//  char CellPhoneOrIPaddress[15];
//  Answer.reserve(32);
//  PhoneNumber.reserve(15);
//  PhoneNumber = JLDRnumero;
//  memset(CellPhoneOrIPaddress, Null, sizeof(CellPhoneOrIPaddress));
//  PhoneNumber.toCharArray(CellPhoneOrIPaddress, PhoneNumber.length() + 1);
//  memset(Message, Null, sizeof(Message));
//  Cde_received = Cde_received.substring(4);
//  Cde_received.toCharArray(Message, Cde_received.length() + 1);
//
//  ReadFlags = Flags;
//  ReadFlags &= (1<<GSMInitialized);
//  if (ReadFlags != 0) {                               // has been initiated
//    Status = http.setPhoneFunc(1);
//    if (Status == true) {
//      Status = MySMS.initSMS();
//      if (Status == true) Serial.println(F("SMS initialization is OK"));
//      else Serial.println(F("SMS initialization is wrong"));
//      Answer = MySMS.list(true);
//      Serial.print(F("List of SMS indices: "));
//      Serial.println(Answer);
//      Status = MySMS.send(&CellPhoneOrIPaddress[0], &Message[0]);
//      if (Status == true) {
//        Serial.print(F("[SMS] Recipient: "));
//        Serial.println(PhoneNumber);
//        Serial.print(F("[SMS] Message: "));
//        Serial.println(Cde_received);
//      }
//    }
//  }
//}
/**********************************************************************************************************************************/
/* Function to get the number of character in an array. Stopped until it detects the Null character '\0'.                         */
/* Function has been checked several times.                                                                                       */
/**********************************************************************************************************************************/
uint8_t GetNbrOfCharForGSM(char *ptr_lcl) {
  uint8_t n = 0;
  do {
    n++;
  } while (*(ptr_lcl++) != '\0');
  n -= 1;
  return n;
}
/**********************************************************************************************************************************/
/* Function to force a hardware reset.                                                                                            */
/**********************************************************************************************************************************/
void software_Reset() {
  asm volatile ("  jmp 0");
}
/********************************************************************************************************/
/* Function to display the content of an array until it encounters a Null character.                    */
/********************************************************************************************************/
void DisplayArrayContent(char *lclptr, boolean CRLF) {
  do {
    Serial.print(*(lclptr++));
  } while (*lclptr != Null);
  if (CRLF == true) Serial.println();
}
/********************************************************************************************************/
/* Function to get an answer from operator using terminal. This function waits a mandatory answer from  */
/* the terminal. The short answer is 'Y' (Yes) or 'N' (No) but this function also would be useful to    */
/* retrieve other long response.                                                                        */
/********************************************************************************************************/
void FillShortAnswerArray() {
  uint8_t k = 0;
  int InComingByte = Null;
  memset(Answer, Null, sizeof(Answer));         // global array
  do {
    if (Serial.available() != 0) {
      InComingByte = Serial.read();
      Answer[k++] = (char)InComingByte;
      if (InComingByte == '\n' || InComingByte == '\r') break;      // We wait the mandatory answer of the operator Yes or No
    }
  } while (1);
  Answer[k] = Null;
  Serial.print(F("Your answer is: "));
  Serial.println(Answer);
}
/********************************************************************************************************/
/* Function to test the status of GSM using the AT command "AT+CPIN?" using the method pinStatus().     */
/********************************************************************************************************/
void TestPINmode() {
  unsigned int PINstatus = 8;
  String PinNumber;                                     // Pin code
  boolean Status;
  PinNumber.reserve(5);
  PinNumber = PinNumberInUse;
  memset(PinNumberArray, Null, sizeof(PinNumberArray));
  PinNumber.toCharArray(PinNumberArray, PinNumber.length() + 1);
  PINstatus = http.pinStatus();                         // non blocking function
  Serial.print(F("[SIM800L] Pin code output: "));
  Serial.println(PINstatus, DEC);
  switch (PINstatus) {
    case (unsigned int)0:
      Serial.println(F("[SIM800L] The Mobile termination is ready to be used"));
      break;
    case 1:
      Serial.println(F("[SIM800L] The Mobile termination is waiting the PIN code"));
      Status = http.enterPinCode(&PinNumberArray[0]);
      if (Status == true) Serial.println(F("[SIM800L] The phone has been unlocked using PIN code"));
      else {
        Serial.println(F("[SIM800L] The PIN code does not match the stored code"));
        Serial.println(F("[SIM800L] Warning, you have only 2 tries before blocking SIM card"));
      }
      break;
    case 2:
      Serial.println(F("[SIM800L] The Mobile termination is waiting the PUK code"));
      break;
    case 3:
      Serial.println(F("[SIM800L] The Mobile termination is waiting for antitheft SIM code"));
      break;
    case 4:
      Serial.println(F("[SIM800L] The Mobile termination is waiting for antitheft PUK code"));
      break;
    case 5:
      Serial.println(F("[SIM800L] The Mobile termination is waiting the PIN2 code"));
      break;
    case 6:
      Serial.println(F("[SIM800L] The Mobile termination is waiting the PUK2 code"));
      break;
    default:
      break;    
  }
}








/* END FILE */
