// Author: Jean-Louis Druilhe (jean-louis.druilhe@univ-tlse3.fr)
// NOTE: 
//
// Global System for Mobile communications Arduino Shield V2
// General Packet Radio Service

#include      "Gsm_ShieldV2.h"

/******************************* Local variables used by this module *******************************/
char                  LocalCharArray[32];
char                  URL_EconectServer[URL_length];
GSMProbeMeasures_t    MyGSMMeasures;
char                  FloatToAsciiArray[10];
char                  CellPhoneOrIPaddress[15];
char                  PinNumberArray[5];
char                  AccessPointNameArray[12];

/******************************* External variables *******************************/
extern volatile uint16_t    cmpt1, cmpt2, cmpt3, cmpt4, cmpt5, cmpt_5ms;                  // Timers
extern volatile uint8_t     compt1, compt2, compt3, compt4, compt5, cmpt_100us;           // Timers
extern volatile uint8_t     Flags;

/* Class instances which are objects with public methods and private attributes */
GSM                                 gsmAccess(VerboseAnswer);
GPRS                                GPRS_Services;
GSMClient                           LinkServer;
GSMModem                            ThisModem;            // #define GSMModem GSM3ShieldV1ModemVerification => getIMEI()
GSMScanner                          ScannerNetworks;      // #define GSMScanner GSM3ShieldV1ScanNetworks
GSM3ShieldV1DirectModemProvider     GSM_Ressources;
GSM3ShieldV1ModemVerification       GSM_ModemVerif;
GSM_SMS                             sms;                  // #define GSM_SMS GSM3SMSService

/**********************************************************************************************************************************/
/* Function to read the IP address of the server.                                                                                 */
/**********************************************************************************************************************************/
void ReadIPSeverAddress() {
  String LocalCde;
  char *Lcl_ptr;
  LocalCde.reserve(20);
  LocalCde = ServerOVH_Address;
  memset(LocalCharArray, Null, sizeof(LocalCharArray));
  LocalCde.toCharArray(LocalCharArray, LocalCde.length() + 1);
  Lcl_ptr = &LocalCharArray[0];
  DividerGSM_ShiledV2(80, true, '-');
  Serial.print(F("IP address is: "));
  do {
    Serial.print(*(Lcl_ptr++));
  } while (*Lcl_ptr != '\0');
  Serial.println();
  DividerGSM_ShiledV2(80, true, '-');
}
/****************************************************************************************************/
/* Local divider (GSM_ShiledV2)                                                                     */
/****************************************************************************************************/
void DividerGSM_ShiledV2(uint8_t nbr_carac, boolean CRLFChar, char caract) {
  uint8_t i;
  for (i = 0; i < nbr_carac; i++) Serial.print(caract);
  if (CRLFChar == true) Serial.println();
}
/****************************************************************************************************/
/* Function to initialize the GSM shield plugged on a MEGA card.                                    */
/* GSM gsmAccess; GPRS GPRS_Services; GSMClient LinkServer;                                         */
/****************************************************************************************************/
boolean initGSMShield() {
  uint8_t counter = 0;
  boolean GSM_Present = false;
  String AccesPointName;
  String PinNumber;
  AccesPointName.reserve(12);
  PinNumber.reserve(5);
  AccesPointName = SFR_WEBPHONE;
  PinNumber = PinNumberSFR8697;
  memset(PinNumberArray, Null, sizeof(PinNumberArray));
  PinNumber.toCharArray(PinNumberArray, PinNumber.length() + 1);
  DividerGSM_ShiledV2(80, true, '-');
  Serial.println(F("Starting the web client services..."));
  Serial.print(F("Initialization pending"));
  while (!GSM_Present) {
    if((gsmAccess.begin(&PinNumberArray[0]) == GSM_READY) &        \
      (GPRS_Services.attachGPRS(SFR_WEBPHONE, GPRS_Login, GPRS_Password) == GPRS_READY)) {
      GSM_Present = true;
    } else {
      counter++;
      compt1 = 0;                     // 5 ms timer
      Serial.print(".");
      //do { } while (compt1 < 40);
      if (counter >= 25) break;       // 40 x 5 ms x 25 about 5 seconds delay
    }
  }
  DividerGSM_ShiledV2(80, true, '-');
  return GSM_Present;
}
/****************************************************************************************************/
/* Second function to initialize the GSM shield plugged on a MEGA card.                             */
/* GSM gsmAccess(VerboseAnswer); GPRS GPRS_Services; GSMClient LinkServer;                          */
/* enum GSM3_NetworkStatus_t { ERROR, IDLE, CONNECTING, GSM_READY, GPRS_READY,                      */
/* TRANSPARENT_CONNECTED, OFF}; (GSM3MobileAccessProvider.h)                                        */
/****************************************************************************************************/
boolean initGSMShield2() {
  char *LclPtr;
  uint8_t counter = 0;
  boolean Status;
  GSM3_NetworkStatus_t State;
  boolean GSM_Present = false;
  String AccesPointName;
  String PinNumber;
  AccesPointName.reserve(12);
  PinNumber.reserve(5);
  //AccesPointName = SFR_WEBPHONE;            // #define SFR_WEBPHONE "sl2sfr"
  //AccesPointName = SFR_Connect_Web;         // => SMS OK
  AccesPointName = OfficialSFRapn;
  PinNumber = PinNumberSFR8697;
  memset(AccessPointNameArray, Null, sizeof(AccessPointNameArray));
  AccesPointName.toCharArray(AccessPointNameArray, AccesPointName.length() + 1);
  memset(PinNumberArray, Null, sizeof(PinNumberArray));
  PinNumber.toCharArray(PinNumberArray, PinNumber.length() + 1);
  
  DividerGSM_ShiledV2(80, true, '-');
  Serial.println(F("*** Starting the web client services ***"));
  Serial.println("[GSM/GPRS/Modem Shield] Connecting to GSM network...");
  Serial.print(F("Access Point Name used: "));
  LclPtr = &AccessPointNameArray[0];
  do {
    Serial.print(*(LclPtr++));
  } while (*LclPtr != '\0');
  Serial.println();
  Serial.print(F("Pin number: "));
  LclPtr = &PinNumberArray[0];
  do {
    Serial.print(*(LclPtr++));
  } while (*LclPtr != '\0');
  Serial.println('\n');
  
  Serial.print(F("[GSM Shield] Initialization pending"));
  cmpt1 = 0;
  Flags |= (1<<Flag0);                                // for a mandatory interruption 
  State = gsmAccess.begin(&PinNumberArray[0]);        // that is a blocking function
  Flags &= ~(1<<Flag0);
  if (State == GSM_READY) {
    Serial.println(F("\n[GSM Shield] GSM initialized\n"));
    GSM_Present = true;
  } else if (State == ERROR) Serial.println(F("\n[GSM Shield] GSM error, not present or is faulty\n"));
  else Serial.println(F("\n[GSM Shield] Other action in progress\n"));
  
  Serial.print(F("[GPRS Shield] Initialization pending"));
  cmpt1 = 0;
  Flags |= (1<<Flag0);
  State = GPRS_Services.attachGPRS(&AccessPointNameArray[0], GPRS_Login, GPRS_Password, true);    // GSM3ShieldV1DataNetworkProvider class
  Flags &= ~(1<<Flag0);
  if (State == GPRS_READY) Serial.println(F("\n[GPRS Shield] GPRS initialized\n"));
  else if (State == ERROR) Serial.println(F("\n[GPRS Shield] GPRS error, not present or is faulty\n"));
  else if (State == CONNECTING) {
    Serial.println(F("\n[GPRS Shield] GPRS is connecting...\n"));
    software_Reset();
  } else if (State == TRANSPARENT_CONNECTED) Serial.println(F("\n[GPRS Shield] GPRS in transparent transmission mode\n"));
  else Serial.println(F("\n[GPRS Shield] Other action in progress\n"));

//  Serial.print(F("[Modem] Initialization pending"));
//  cmpt1 = 0;
//  Flags |= (1<<Flag0);
//  Status = ThisModem.begin();   // GSMModem ThisModem; /* #define GSMModem GSM3ShieldV1ModemVerification => getIMEI() */
//  Flags &= ~(1<<Flag0);
//  if (Status == 0) Serial.println(F("\n[Modem] No modem answer\n"));
//  else Serial.println(F("\n[Modem] Modem can accept AT command\n"));

  DividerGSM_ShiledV2(80, true, '-');
  return GSM_Present;
}
/**********************************************************************************************************************************/
/* Function to gather all text topics including IP address to make an URL for GPRS.                                               */
/* frame received: URL_EntirePacket(AllMeas.DO_FloatValue, AllMeas.pH_FloatValue, AllMeas.ORP_FloatValue, AllMeas.EC_FloatValue,  */
/* AllMeas.Temp_FloatValue, AllMeas.Lux_FloatValue);                                                                              */
/* http://145.239.199.14/cgi-bin/econect/receive_data.py?location=aquacosme1&temp_eau=18.73&ph=7.21&int_lumineuse=799.3&oxygene=16.2&conductivite=123.4&redox=-24.71
/**********************************************************************************************************************************/
void URL_EntirePacket(float DO, float pH, float ORP, float EC, float DS18, float Lux) {
  MyGSMMeasures.DO_GSMValue = DO;
  MyGSMMeasures.pH_GSMValue = pH;
  MyGSMMeasures.ORP_GSMValue = ORP;
  MyGSMMeasures.EC_GSMValue = EC;
  MyGSMMeasures.Temp_GSMValue = DS18;
  MyGSMMeasures.Lux_GSMValue = Lux;

  int Status;
  int Available;
  char MyChar;
  String LocalURL;
  String AsciiFloat;
  String OVH_IP;
  char *Lcl_ptr;
  uint8_t NbrChar;
  LocalURL.reserve(URL_length);
  AsciiFloat.reserve(10);
  OVH_IP.reserve(15);

  LocalURL.concat(StartFrame);                  // "GET /"
  LocalURL.concat(EconectReceiveDatas);         // "/cgi-bin/econect/receive_data.py?location=aquacosme1"
  LocalURL.concat(TempDS18B20Topic);            // "&temp_eau="
  ConvFloatToStringWithSign(MyGSMMeasures.Temp_GSMValue, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
  LocalURL.concat(pHTopic);
  ConvFloatToStringWithSign(MyGSMMeasures.pH_GSMValue, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
  LocalURL.concat(VEML7700Topic);
  ConvFloatToStringWithSign(MyGSMMeasures.Lux_GSMValue, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
  LocalURL.concat(DOTopic);
  ConvFloatToStringWithSign(MyGSMMeasures.DO_GSMValue, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
  LocalURL.concat(CondTopic);
  ConvFloatToStringWithSign(MyGSMMeasures.EC_GSMValue, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
  LocalURL.concat(ORPTopic);
  ConvFloatToStringWithSign(MyGSMMeasures.ORP_GSMValue, 2, FloatToAsciiArray);
  Lcl_ptr = &FloatToAsciiArray[0];
  AsciiFloat = String(Lcl_ptr);
  LocalURL.concat(AsciiFloat);
  LocalURL.concat(EndFrame);
  LocalURL.concat("\r\n");
  LocalURL.concat(HeaderHost);                  // "Host: "
  LocalURL.concat(ServerOVH_Address);           // "145.239.199.14"
  LocalURL.concat("\r\n");
  LocalURL.concat(CloseFrame);
  LocalURL.concat("\r\n\r\n");
  
//  LocalURL = httpGETcde2;
//  LocalURL.concat(StartURL);
//  LocalURL.concat(ServerOVH_Address);
//  LocalURL.concat(EconectReceiveDatas);
//  LocalURL.concat(TempDS18B20Topic);
//  ConvFloatToStringWithSign(MyGSMMeasures.Temp_GSMValue, 2, FloatToAsciiArray);
//  Lcl_ptr = &FloatToAsciiArray[0];
//  AsciiFloat = String(Lcl_ptr);
//  LocalURL.concat(AsciiFloat);
//  LocalURL.concat(pHTopic);
//  ConvFloatToStringWithSign(MyGSMMeasures.pH_GSMValue, 2, FloatToAsciiArray);
//  Lcl_ptr = &FloatToAsciiArray[0];
//  AsciiFloat = String(Lcl_ptr);
//  LocalURL.concat(AsciiFloat);
//  LocalURL.concat(VEML7700Topic);
//  ConvFloatToStringWithSign(MyGSMMeasures.Lux_GSMValue, 2, FloatToAsciiArray);
//  Lcl_ptr = &FloatToAsciiArray[0];
//  AsciiFloat = String(Lcl_ptr);
//  LocalURL.concat(AsciiFloat);
//  LocalURL.concat(DOTopic);
//  ConvFloatToStringWithSign(MyGSMMeasures.DO_GSMValue, 2, FloatToAsciiArray);
//  Lcl_ptr = &FloatToAsciiArray[0];
//  AsciiFloat = String(Lcl_ptr);
//  LocalURL.concat(AsciiFloat);
//  LocalURL.concat(CondTopic);
//  ConvFloatToStringWithSign(MyGSMMeasures.EC_GSMValue, 2, FloatToAsciiArray);
//  Lcl_ptr = &FloatToAsciiArray[0];
//  AsciiFloat = String(Lcl_ptr);
//  LocalURL.concat(AsciiFloat);
//  LocalURL.concat(ORPTopic);
//  ConvFloatToStringWithSign(MyGSMMeasures.ORP_GSMValue, 2, FloatToAsciiArray);
//  Lcl_ptr = &FloatToAsciiArray[0];
//  AsciiFloat = String(Lcl_ptr);
//  LocalURL.concat(AsciiFloat);

  memset(URL_EconectServer, Null, sizeof(URL_EconectServer));
  LocalURL.toCharArray(URL_EconectServer, LocalURL.length() + 1);
  NbrChar = GetNbrOfCharForGSM(&URL_EconectServer[0]);
  Lcl_ptr = &URL_EconectServer[0];
  DividerGSM_ShiledV2(140, true, '-');
  #ifdef messagesON
    Serial.print(F("URL for server or Json frame: "));
    do {
      Serial.print(*(Lcl_ptr++));
    } while (*Lcl_ptr != '\0');
    Serial.println();
    Serial.print(F("Length of the frame in number of characters: "));
    Serial.println(NbrChar, DEC);
  #endif
  
  memset(CellPhoneOrIPaddress, Null, sizeof(CellPhoneOrIPaddress));
  OVH_IP = ServerOVH_Address;                   // "145.239.199.14"
  OVH_IP.toCharArray(CellPhoneOrIPaddress, OVH_IP.length() + 1);
  #ifdef messagesON
    Serial.print(F("Check the IP address: "));
    Lcl_ptr = &CellPhoneOrIPaddress[0];
    do {
      Serial.print(*(Lcl_ptr++));
    } while (*Lcl_ptr != Null);
    Serial.println();
  #endif
  Status = LinkServer.connect(&CellPhoneOrIPaddress[0], (uint16_t)ServerPort);      // GSMClient LinkServer;
  if (Status != 0) {
    LinkServer.write(&URL_EconectServer[0], NbrChar);
    //LinkServer.print(&URL_EconectServer[0]);
    LinkServer.endWrite();
    Available = LinkServer.available();
    while (Available != 0) {
      MyChar = LinkServer.read();
      Serial.print(MyChar);
    }
//    if (!LinkServer.available() && !LinkServer.connected()) {
//      Serial.println();
//      Serial.println("disconnecting.");
//      LinkServer.stop();
//    }
    LinkServer.stop();
  } else Serial.println(F("HTTP command is invalid"));
  DividerGSM_ShiledV2(140, true, '-');
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
//  MyGSMFloat_t LocalFloat;
//  LocalFloat.value = ConvertFloat;
  uint32_t IEEE754Representation = 0;
  uint32_t IntegerResult;
  char Scratch_tab[10];       // to convert uint32_t in ASCII format
  uint8_t NbrChar;
  uint8_t CommaPosition;
  
//  for (k = 4; k > 0; k--) {
//    IEEE754Representation |= LocalFloat.byte_value[k - 1];
//    if (k != 1) IEEE754Representation <<= 8;
//  }
//  if (IEEE754Representation & Sign_Mask) {        // #define Sign_Mask 0x80000000
//    *(DestArray++) = '-';
//    LocalFloat.value *= -1.0;                     // positive number
//  }

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
  int Status;
  int InitGSM;
  String IMEI;
  String Answer;
  IMEI.reserve(20);
  Answer.reserve(10);
  GSM_Ressources.begin();                       // GSM3ShieldV1DirectModemProvider GSM_Ressources;
  InitGSM = GSM_ModemVerif.begin();             // GSM3ShieldV1ModemVerification GSM_ModemVerif;
  Serial.print(F("Modem Verification = ")); Serial.println(InitGSM, DEC);
  delay(1000);
  Serial.println("Test: " + GSM_Ressources.writeModemCommand("AT", 200));

//  ScannerNetworks.begin();
//  Serial.println(ScannerNetworks.getCurrentCarrier());
  Status = ThisModem.begin();
  if (Status == 0) Serial.println(F("[Error] The modem is out of work..."));
  else {
    IMEI = ThisModem.getIMEI();
    IMEI.replace("\n", "");
    if (IMEI != "") {
      Serial.print(F("IMEI code: "));
      Serial.println(IMEI);
    }
  }

  Answer = GSM_Ressources.writeModemCommand("AT", 500);                // normaly we wait an acknowledge as "OK"
  Serial.print(F("Answer from GSM module: ")); Serial.println(Answer);
  Answer = GSM_Ressources.writeModemCommand("ATI", 500); Serial.println(Answer);
}
/**********************************************************************************************************************************/
/* Function to send a SMS message to a mobile phone.                                                                              */
/* The control command looks like 'sms_'<My text>                                                                                 */
/**********************************************************************************************************************************/
void SendSMS(String Cde_received) {
  int NbrChar;
  String PhoneNumber;
  char Message[50];
  PhoneNumber.reserve(15);
  PhoneNumber = JLDRnumero;
  memset(CellPhoneOrIPaddress, Null, sizeof(CellPhoneOrIPaddress));
  PhoneNumber.toCharArray(CellPhoneOrIPaddress, PhoneNumber.length() + 1);
  memset(Message, Null, sizeof(Message));
  Cde_received = Cde_received.substring(4);
  Cde_received.toCharArray(Message, Cde_received.length() + 1);
  sms.beginSMS(&CellPhoneOrIPaddress[0]);
  //sms.beginSMS(CellPhoneOrIPaddress);
  NbrChar = sms.print(&Message[0]);
  sms.endSMS();
  Serial.print(F("[SMS] Message Recipient: "));
  Serial.println(PhoneNumber);
  Serial.print(F("[SMS] Message: "));
  Serial.println(Message);
  Serial.print(F("[SMS] number of characters: "));
  Serial.println(NbrChar, DEC);
  
}
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







/* END FILE */
