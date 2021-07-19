// Author: Jean-Louis Druilhe (jean-louis.druilhe@univ-tlse3.fr)
// NOTE: 
//
// Probes which can be connected to the multiparameter probe according the communication bus used:
// I2C => Dissoled Oxygen (DO), pH (pH), Oydo Reduction Potential (ORP), Conductivity (EC), Luminosity (Lux), Platinum Resistance Thermometer (RTD) 
// OneWire => DS18B20 (Temp)


#include      "Functions.h"
#include      "veml7700_functions.h"                      // necessary when there is a global measure applied for all probes

/* Local variables used by this module */
volatile uint16_t           cmpt1, cmpt2, cmpt3, cmpt4, cmpt5, cmpt_5ms;
volatile uint8_t            compt1, compt2, compt3, compt4, compt5, cmpt_100us;
volatile uint8_t            FlagsReader;
uint8_t                     BusyTimeForWatchdog, Flags;   // global variables used for GSM and tied to the timers and the watchdog
uint8_t                     WatchdogCounter = 0;

uint8_t                     scratch_8bitsFunc;
uint16_t                    scratch_16bitsFunc;
uint32_t                    scratch_32bitsFunc;
char                        sensordata[32];               // 32 characters array to hold incoming data from the sensors
uint8_t                     sensor_bytes_received;        // We need to know how many characters bytes have been received
I2CProbesConnected_t        I2C_ProbesOfRaft;             // all I2C probes connected to the instrument
boolean                     DS18B20IsPresent;             // only for the oneWire probe DS18B20 connected or no

char                        *ptr_String;                  // pointer to allow conversion to String declarations
char                        ConvStringAsArray[32];
char                        StampCmd[32];
char                        TabASCII[20];
char                        TerminalCde[20];
char                        MyAnswer[4];                  // used for short answers
char                        FloatAscii[6];
char                        Uint16DecAscii[5];
char                        Uint32DecAscii[10];

char                        ConvUintxx_tToAsciiChar[20];
I2CAddresses_t              TokenAddressInProgress;
token_t                     ProbeSelected;
uint8_t                     OneWireAddress[8];
char                        *ptr_StampType;                 // global pointer only used by scan function
//String                      stamp_type;                   // hold the name (type of the stamp)
PowerSwitches_t             MyPowerSwitches;

/* External variables */

/* Class instances which are objects with public methods and private attributes */
OneWire Temp_Probe(DS18B20Temp);                // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature DS18B20Probe(&Temp_Probe);    // Pass our oneWire reference to Dallas Temperature
#define MyI2CDevice Wire
Adafruit_ADS1115 MyADC;                         // instantiate an object

/* #########################################Interruptions########################################## */
/****************************************************************************************************/
/* Programmes d'interrution des 3 compteurs. Le compteur0 est réservé par plusieurs bibliothèques.  */
/* Timers utilisés : Timer1 (16 bits, 5 ms), Timer2 (8 bits, 100 µs)                                */
/****************************************************************************************************/
// Timer1 configured for time delay of 5 ms
ISR(TIMER1_COMPA_vect) {
  compt1++;                           // 8 bits
  cmpt1++;                            // 16 bits
  cmpt_5ms++;                         // 16 bits
  FlagsReader = Flags;
  FlagsReader &= (1<<UsingTimer1Interrupt);   // This flag is declared by only one module at the same time
  if (FlagsReader != 0) {
    if (compt1 > 60) {
      compt1 = 0;
      Serial.print(F("."));           // Output on terminal
    }
  }
}
// Compteur 2 programmé pour des temporisations de 100 µs
ISR(TIMER2_COMPA_vect) {
  compt2++;                     // 8 bits
  cmpt2++;                      // 16 bits
  cmpt_100us++;                 // 8 bits
}
// Timer3 configured to define delay time of 200 ms
ISR(TIMER3_COMPA_vect) {
  compt3++;                     // 8 bits
  cmpt3++;                      // 16 bits
  
  FlagsReader = Flags;
  FlagsReader &= (1<<WatchdogDelayArmed);           // Common flag activated according to the need
  if (FlagsReader != 0) {
    Flags &= ~(1<<WatchdogDelayArmed);              // to be here only one time
    MCUSR &= ~(1<<WDRF);                            // WDRF is cleared (MCUSR &= ~(1<<WDRF); ou MCUSR = 0;)
    WDTCSR |= ((1<<WDCE)|(1<<WDE));                 // mandatory to change the prescaler value
    //WDTCSR = ((1<<WDIE)|(1<<WDP3)|(1<<WDP0));       // ~8s we active the interruption of the watchdog timer
    WDTCSR = ((1<<WDIE)|(1<<WDP2)|(1<<WDP1));       // ~1s we active the interruption of the watchdog timer                       
    asm volatile ("wdr");                           // to start from a zero counter value
  }
  
  FlagsReader = Flags;
  FlagsReader &= (1<<StopTheWatchdogTimer);     // flag activated after the initialization of the GPRS and to stop the WDT
  if (FlagsReader != 0) {
    Flags &= ~(1<<StopTheWatchdogTimer);
    WatchdogCounter = 0;
    MCUSR &= ~(1<<WDRF);                        // WDRF is cleared (MCUSR &= ~(1<<WDRF); ou MCUSR = 0;)
    WDTCSR |= (1<<WDCE);                        // to change the bits as WDE bit and prescaler bits
    WDTCSR = 0x0;
  }
}
// Compteur 4 programmé pour des temporisations de 100 ms
ISR(TIMER4_COMPA_vect) {
  compt4++;                     // 8 bits
  cmpt4++;                      // 16 bits
}
// Compteur 5 programmé pour des temporisations longues de 50 ms
ISR(TIMER5_COMPA_vect) {
  compt5++;
  cmpt5++;
}
// Watchdog timer interruption for the initialization of GPRS and GSM
ISR(WDT_vect) {
  WatchdogCounter++;
  if (WatchdogCounter == BusyTimeForWatchdog) {   // BusyTimeForWatchdog is loaded with a specific value before calling a blocking function
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE);                          // to change the bits as WDE bit and prescaler bits
    WDTCSR = 0x0; 
    asm volatile ("jmp 0");                       // Software reset
  } else {
    MCUSR &= ~(1<<WDRF);
  }
}
/* ####################################Fin des interruptions####################################### */

//* #########################################Fonctions########################################## */
/****************************************************************************************************/
/* Fonction pour initialiser les 6 timers du microcontrôleur ATmega2560. Le programme principal     */
/* doit charger l'entier Ctrl_Flags pour déterminer les compteurs utilisés. Par convention :        */
/* Timer0 (8 bits), temporisation : 1 ms, mode CTC, prescaler = 64 => OCR0A = 250                   */                                       
/* Timer1 (16 bits), temporisation : 5 ms, mode CTC, prescaler = 8 => OCR1A = 0x2710 (10000)        */
/* Timer2 (8 bits), temporisation : 100 µs, mode CTC, prescaler = 8 => OCR2A = 0xC8 (200)           */
/* Timer3 (16 bits), temporisation : 10 ms, mode CTC, prescaler = 64 => OCR3A = 0x09C4 (2500)       */
/* --- Timer3 will not have to be used while some others ressources use it. ---                     */
/* Timer4 (16 bits), temporisation : 100 ms, mode CTC, prescaler = 64 => OCR4A = 0x61A8 (25000)     */
/* --- Timer4 will not have to be used while some others ressources use it. ---                     */
/* Timer5 (16 bits), temporisation : 50 ms, mode CTC, prescaler = 256 => OCR5A = 0x0C35 (3125)      */
/* Timer0 => used by LiquidCrystal_I2C library. The Timer0 will not have to be configured.          */
/* Above all the clock of the timer0 will not have to be stopped and is applicable only for him.    */
/****************************************************************************************************/
void Init_Timers(uint8_t drapeauxTimers, uint8_t time_Timer0, uint16_t time_Timer1, uint8_t time_Timer2, uint16_t time_Timer3, uint16_t time_Timer4, uint16_t time_Timer5) {
  uint8_t test_drapeaux;
  test_drapeaux = (drapeauxTimers & (1 << Timer0_ON)); // le seul qui ne doit pas être modifié car il est utilisé par la bibliothèque LiquidCrystal_I2C.h
  if (test_drapeaux != 0) {
    /* compteur 0 */                        // ne pas être utilisé avec l'afficheur I2C
    TCCR0A = 0;                             // (Par défaut) WGM00, WGM01, COM0B0, COM0B1, COM0A0, COM0A1 = 0
    TCCR0A |= (1 << WGM01);                 // CTC (Mode 2) => Clear Timer on Compare match
    TCCR0B = 0;                             // WGM02 = 0
    TCCR0B |= ((1 << CS00) | (1 << CS01));  // division par 64 => 250 KHz
    OCR0A = time_Timer0;                    // temporisation de 1 ms avec 250 (0xFA)
    TIMSK0 |= (1 << OCIE0A);                // vecteur d'interruption
    TCNT0 = 0;
  } else {
    //TCCR0B &= ~((1<<CS00)|(1<<CS01)|(1<<CS02));         // ne doit pas être bloqué car les méthodes de la bibliothèque de l'afficheur ne l'activent pas
  }
  test_drapeaux = (drapeauxTimers & (1 << Timer1_ON));    // compteur1 16 bits configuré pour 5 ms
  if (test_drapeaux != 0) {
    /* compteur 1 => WGM10=0, WGM11=0, WGM12=1, WGM13=0 => CTC */
    TCCR1A = 0;                             // WGM10 et WGM11 sont à  0
    TCCR1B = (1 << WGM12);                  // Mode 4 du timer1
    TCCR1B |= (1 << CS11);                  // prescaler = 8 et WGM13 = 0 => 2 MHz
    TIMSK1 |= (1 << OCIE1A);                // autorisation d'interruption par comparaison : désigne un type d'interruption
    TCNT1 = 0;
    scratch_16bitsFunc = time_Timer1;
    OCR1AH = (uint8_t)(scratch_16bitsFunc >> 8);    // 0x2710 = 10000
    OCR1AL = (uint8_t)time_Timer1;              // tempo = 10000 x 1/(16MHz/8) = 5 ms
  } else {
    TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12)); // blocage du compteur
  }
  test_drapeaux = (drapeauxTimers & (1 << Timer2_ON));   // compteur2 8 bits configuré pour 100 µs
  if (test_drapeaux != 0) {
    // compteur 2 (configuration CTC) compteur 8 bits
    TCCR2A |= (1 << WGM21);   // CTC (Mode 2)
    TCCR2B = 0;               // WGM02 = 0, CS20 = 0, CS21 = 0, CS22 = 0
    TCCR2B |= (1 << CS21);    // division par 8 => 2 MHz
    OCR2A = time_Timer2;      // temporisation de 100 µs => valeur décimale 200 (0xC8)
    TIMSK2 |= (1 << OCIE2A);  // vecteur d'interruption
    TCNT2 = 0;
  } else {
    TCCR2B &= ~((1 << CS20) | (1 << CS21) | (1 << CS22));   // blocage du compteur
  }
  test_drapeaux = (drapeauxTimers & (1 << Timer3_ON));      // compteur3 16 bits configuré pour 200 ms
  if (test_drapeaux != 0) {
    TCCR3B |= (1 << WGM32);                                 // WGMn2 doit être le seul drapeau activé (mode 4 => CTC)
    TCCR3A = 0;
    TCCR3B |= ((1 << CS31) | (1 << CS30));                  // division par 64 => 250 Khz
    scratch_16bitsFunc = time_Timer3;                       // 50000 à convertir
    OCR3AH = (uint8_t)(scratch_16bitsFunc >> 8);            // 0x09C4 = 2500
    OCR3AL = (uint8_t)time_Timer3;
    TIMSK3 |= (1 << OCIE3A);                                // autorisation d'interruption par comparaison
    TCNT3 = 0;                                              // initialisation du compteur non nécessaire
  } else {
    //TCCR3B &= ~((1 << CS30) | (1 << CS31) | (1 << CS32)); // blocage du compteur qui sera activé par d'autres ressources
  }
  test_drapeaux = (drapeauxTimers & (1 << Timer4_ON));      // compteur4 16 bits configuré pour 100 ms
  if (test_drapeaux != 0) {
    TCCR4B |= (1 << WGM42);                                 // WGMn2 doit être le seul drapeau activé (mode 4 => CTC)
    //TCCR4A = 0;
    TCCR4B |= ((1 << CS41) | (1 << CS40));                  // division par 64 => 250 Khz
    scratch_16bitsFunc = time_Timer4;                       // 25000 à convertir pour 100 ms
    OCR4AH = (uint8_t)(scratch_16bitsFunc >> 8);            // 0x61A8 = 25000
    OCR4AL = (uint8_t)time_Timer4;
    TIMSK4 |= (1 << OCIE4A);                                // autorisation d'interruption par comparaison
    TCNT4 = 0;                                              // initialisation du compteur non nécessaire
  } else {
    //TCCR4B &= ~((1 << CS40) | (1 << CS41) | (1 << CS42)); // blocage du compteur à l'initialisation qui sera utilisé par d'autres ressources
  }
  test_drapeaux = (drapeauxTimers & (1 << Timer5_ON));      // compteur5 16 bits configuré pour 50 ms
  if (test_drapeaux != 0) {
    TCCR5B |= (1 << WGM52);                                 // WGMn2 doit être le seul drapeau activé (mode 4 => CTC)
    //TCCR5A = 0;                                             // par défaut
    TCCR5B |= (1 << CS52);                                  // division par 256 => 62,5 Khz
    scratch_16bitsFunc = time_Timer5;                       // 3125 à convertir pour 50 ms
    OCR5AH = (uint8_t)(scratch_16bitsFunc >> 8);            // 0x0C35 = 3125
    OCR5AL = (uint8_t)time_Timer5;
    TIMSK5 |= (1 << OCIE5A);                                // autorisation d'interruption par comparaison
    TCNT5 = 0;                                              // initialisation du compteur non nécessaire
  } else {
    TCCR5B &= ~((1 << CS50) | (1 << CS51) | (1 << CS52)); // blocage du compteur
  }
}
/********************************************************************************************************************************/
/* Polling function to check the presence of all I2C devices.                                                                   */
/* Return a complex type as a structure. I2C_ProbesOfRaft is a gloabal variable which inform the presence of each I2C probes.   */
/********************************************************************************************************************************/
I2CProbesConnected_t scani2c() {                            // Scan for all I2C devices
  uint8_t Nbr_stamps, add_stamp;
  I2C_ProbesOfRaft.DO_Probe = false;                        // necessary to reset all devices encountered
  I2C_ProbesOfRaft.ORP_Probe = false;                       // I2CProbesConnected_t I2C_ProbesOfRaft; (global variable)
  I2C_ProbesOfRaft.pH_Probe = false;
  I2C_ProbesOfRaft.EC_Probe = false;
  I2C_ProbesOfRaft.RTD_Probe = false;
  I2C_ProbesOfRaft.VEML7700_Probe = false;
  Serial.println(F("Starting  I2C scan..."));
  Nbr_stamps = 0;
  for (add_stamp = 1; add_stamp < 128; add_stamp++) {
    if (check_i2c_connection(add_stamp) == 0) {             // if I2C device is present (twi.cpp) (the only function called)
      Nbr_stamps++;
      Serial.print(F("\t\t\t\t\uFFED I2C Channel: 0x"));    // store string in flash memory
      Serial.println(add_stamp, HEX);
      Serial.print(F("\t\t\t\t\uFFED Atlas Scientific device: "));
      DisplayArrayContentFunctions(ptr_StampType, true);    // ptr_StampType = global pointer sended by parseInfo function
    }
  }
  Divider(80, true, '-');
  Serial.println(F("SCAN COMPLETE"));
  Serial.print(F("Discovered I2C peripherals: "));
  Serial.println(Nbr_stamps, DEC);
  Divider(80, true, '-');
  return I2C_ProbesOfRaft;                                  // global variable
}
/********************************************************************************************************************************/
/* All I2C devices have to be found and identified using the name of the probe.                                                 */
/* MyI2CDevice is an object of the Wire class.                                                                                  */
/********************************************************************************************************************************/
uint8_t check_i2c_connection(uint8_t add_i2c) {     // check selected i2c channel/address. verify that it's working by requesting info about the stamp
  uint8_t status_transmit;
  String AtlasCde;                                  // short command
  AtlasCde.reserve(4);
  MyI2CDevice.beginTransmission(add_i2c);           // START followed by a STOP : https://www.arduino.cc/en/Reference/WireBeginTransmission initie une transmission I2C avec l'adresse spécifiée
  status_transmit = MyI2CDevice.endTransmission();  // Bytes sended after test are 0 for success, 1-4 for errors https://www.arduino.cc/en/Reference/WireEndTransmission
  if (status_transmit == 0) {                       // if success => uint8_t twi_writeTo(uint8_t address, uint8_t *data, uint8_t length, uint8_t wait, uint8_t sendStop) {...}
    AtlasCde = Cmd_I;                                         // 'I'
    memset(StampCmd, Null, sizeof(StampCmd));                 // char StampCmd[32];
    AtlasCde.toCharArray(StampCmd, AtlasCde.length() + 1);    // mandatory increase of one character filled with '\0'
    Divider(80, true, '-');
    Serial.println(F("\n** Informations read from the stamp **"));
    I2C_call(StampCmd, (I2CAddresses_t)add_i2c, AtlasCde.length());   // sends a control command and retrieve a data frame stored in StampCmd array
    ptr_StampType = parseInfo((I2CAddresses_t)add_i2c);               // sortie de la méthode au premier return rencontré (on identifie les adresses I2C de chaque module)
  }
  return status_transmit;
}
/* **************************************************************************************************************************** */
/* Null character detected to indicate the end of the answser provided by the probe.                                            */
/* some arduinos put padding zeroes in the receiving buffer (up to the number of requested bytes)                               */
/* method Wire : http://docs.bluz.io/reference/wire/                                                                            */
/* Wire.write : https://www.arduino.cc/en/Tutorial/MasterWriter                                                                 */
/* write method from Wire class: write(const uint8_t *data, size_t quantity) => bytes used                                      */
/* **************************************************************************************************************************** */
void I2C_call(char *Cde_busI2C, I2CAddresses_t ADDRESS, uint8_t nbr_carCde) {     // function to parse and call I2C commands.
  uint8_t k;
  uint8_t i2c_response_code;        // used to hold the I2C response code.
  uint8_t NbrBytes;                 // We need to know how many characters bytes have been received
  uint8_t byteIndex = 0;

  memset(sensordata, Null, sizeof(sensordata));           // clear sensordata array which is global [32]
  MyI2CDevice.beginTransmission((uint8_t)ADDRESS);        // call the circuit by its ID number : void TwoWire::beginTransmission(uint8_t address) https://www.arduino.cc/en/Reference/WireBeginTransmission
  MyI2CDevice.write(Cde_busI2C, nbr_carCde);              // virtual size_t write(const uint8_t *, size_t); => here is the real transmission of a control command to the Atlas Scietific stamp
  MyI2CDevice.endTransmission();
  #ifdef messagesON
    Serial.print(F("Control command transmitted to the stamp: "));
    for (k = 0; k < nbr_carCde; k++) Serial.print(*(Cde_busI2C++));
  #endif
  Serial.print(F("\n[DEVICE] Control command state: answer pending"));
  NbrBytes = MyI2CDevice.requestFrom((uint8_t)ADDRESS, (uint8_t)20, (uint8_t)1);    // return uint8_t requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
  i2c_response_code = MyI2CDevice.read();                 // first byte read
  sensordata[byteIndex++] = i2c_response_code;            // global array [32]
  Flags |= (1<<WatchdogDelayArmed);
  BusyTimeForWatchdog = BusyTimeForProbes;
  
  if (i2c_response_code == 254) {                         // means the command has not yet been finished calculating
    do {
      cmpt_5ms = 0;                                       // Timer1
      do { } while(cmpt_5ms < 10);                        // time delay of 50 ms for each point character displayed
      Serial.print('.');
      byteIndex = 0;
      NbrBytes = MyI2CDevice.requestFrom((uint8_t)ADDRESS, (uint8_t)20, (uint8_t)1);
      i2c_response_code = MyI2CDevice.read();
      sensordata[byteIndex++] = i2c_response_code;
      if (i2c_response_code != 254) {
        Flags |= (1<<StopTheWatchdogTimer);
        while (MyI2CDevice.available()) {                 // return rxBufferLength - rxBufferIndex;
          sensordata[byteIndex++] = MyI2CDevice.read();   // The answer is acquired here
        }
        sensordata[byteIndex] = Null;
        break;                                            // Outgoing method from the do while loop
      }
    } while(i2c_response_code == 254);
  } else {
    Flags |= (1<<StopTheWatchdogTimer);
    while (MyI2CDevice.available()) {                     // return rxBufferLength - rxBufferIndex;
      sensordata[byteIndex++] = MyI2CDevice.read();
    }
    sensordata[byteIndex] = Null;
  }

  #ifdef messagesON
    Serial.print(F("\n[DEVICE] Stamp state: "));
    switch (i2c_response_code) {                          // switch case based on what the response code is.
      case 1:
        Serial.println(F("success"));
        Serial.print(F("[DEVICE] Frame received: "));
        DisplayFrameFromStamp(&sensordata[0]);
        break;
      case 2:
        Serial.println(F("command failed"));
        break;
      case 255:
        Serial.println(F("no data received"));            // means there is no further data to send.
        break;
      default:
        Serial.println(F("frame from other probes"));     // means different probes than Atlas Scientific.
        break;
    }
  #endif
}
/**********************************************************************************************************************************/
/* To display the correct ASCII characters on terminal. We have to change only the first character of the frame.                  */
/* The character chain use the principle of a pointer conversion to get a String. Used only one time.                             */
/**********************************************************************************************************************************/
void DisplayFrameFromStamp(char *ptr_frame) {
  String MyFrame;
  MyFrame.reserve(20);
  if ((uint8_t)(*ptr_frame) >= 1 || (uint8_t)(*ptr_frame) <= 9) *ptr_frame += 0x30;
  MyFrame = String(ptr_frame);
  Serial.println(MyFrame);
}
/**********************************************************************************************************************************/
/* Function to read module's version using an i2C address.                                                                        */
/* parses the answer to a 'I' command. returns true if answer was parseable, false if not.                                        */
/* All responses begin with the decimal value of 1 when device have done an answer.                                               */
/* PH EZO  . "1?I,pH,1.1" or "1?i,pH,1.0"                                                                                         */
/* ORP EZO . "1?I,ORP,1.0"                                                                                                        */
/* DO EZO  . "1?I,D.O.,1.1" || "1?I,DO,1.7" (. exists in D.O. and DO form)                                                        */
/* EC EZO  . "1?I,EC,1.0"                                                                                                         */
/* RTD EZO . "1?I,RTD,1.2" or "1?i,RTD,2.01"                                                                                      */
/* This function returns a char pointer which represents the type of the probe. She is called only for the 'i' command allowing   */
/* to get informations from the module as the firmware version.                                                                   */
/**********************************************************************************************************************************/
char *parseInfo(I2CAddresses_t StampAddress) {    // parses the answer to an 'i' or 'I' command. returns true if answer was parseable, false if not.
  uint8_t m;
  char *ptr_char;
  //char  stamp_version[5];           // hold the version of the stamp (only used in this function)
  String Answer;
  Answer.reserve(32);               // same dimension as sensordata[]
  String stamp_type;                // hold the name / type of the stamp
  stamp_type.reserve(18);           // reserve string buffer to save some SRAM only here
  //memset(stamp_version, Null, sizeof(stamp_version));         
  ptr_char = &sensordata[0];        // sensordata is a global array and contains the response of the probe following a command Cmd_I ('I') for this function
  Answer = String(ptr_char);
  switch (StampAddress) {
    case DissolvedOxygen_Add:
      stamp_type = F("EZO DO");
      Answer = Answer.substring(7);               // "1?I,DO,2.14"
      I2C_ProbesOfRaft.DO_Probe = true;           // I2CProbesConnected_t I2C_ProbesOfRaft; (global variable)
      break;
    case ORP_Add:
      stamp_type = F("EZO ORP");
      Answer = Answer.substring(8);               // "1?I,ORP,1.0"
      I2C_ProbesOfRaft.ORP_Probe = true;
      break;
    case pH_Add:
      stamp_type = F("EZO pH");
      Answer = Answer.substring(7);               // "1?i,pH,1.98"
      I2C_ProbesOfRaft.pH_Probe = true;
      break;
    case Conductivity_Add:
      stamp_type = F("EZO EC");
      Answer = Answer.substring(7);               // "1?i,EC,2.10"
      I2C_ProbesOfRaft.EC_Probe = true;
      break;
    case RTD_Add:
      stamp_type = F("EZO RTD");
      Answer = Answer.substring(8);               // "1?i,RTD,2.01"
      I2C_ProbesOfRaft.RTD_Probe = true;
      break;
    case VEML7700_Add:
      stamp_type = F("ALS VEML7700");
      Answer = "?.??";
      I2C_ProbesOfRaft.VEML7700_Probe = true;
      break;
    default:
      stamp_type = F("unknown EZO stamp");
      Answer = "?.??";
      break;
  }
  Serial.print(F("\t\t\t\t\uFFED Firmware: "));
  Serial.println(Answer);
  stamp_type.toCharArray(ConvStringAsArray, stamp_type.length() + 1);   // global array ConvStringAsArray[32]
  return &ConvStringAsArray[0];                                         // pointer to allow conversion to String declarations
}
/**********************************************************************************************************************************/
/*                                                        HELP FUNCTION                                                           */
/**********************************************************************************************************************************/
void help(String Cde_received) {
  Cde_received = Cde_received.substring(4);               // to remove the text "help"
  memset(TabASCII, Null, sizeof(TabASCII));               // void *memset(void *str, int c, size_t n)
  Cde_received.toCharArray(TabASCII, Cde_received.length() + 1);
  if (TabASCII[0] == '\0') {
    Divider(140, true, '=');
    //Serial.println("\u03BC");   // µ char
    Serial.println(F("\t\t\t**** INFORMATIONS ****\n"));
    Serial.println(F("- To change the I2C address, it is necessary to specify the name of the selected device before sending a frame control."));
    Serial.println(F("- All commands do not have to be write with the symbol ' which is just a separator between text and digits."));
    Serial.println(F("\t* To select a device available, see below about the control command 'token'"));
    Serial.println(F("\t* The names of the control commands are contained between the character '"));
    Serial.println(F("\t* All digits are contained between symbols < and >\n"));
    Serial.println(F("**** Control frames to change the I2C address of an Atlas Scientific module ****"));
    Serial.println(F("\t\t1) 'token_'<probe>: to select a probe"));
    Serial.println(F("\t\t2) 'change_add'<1 to 127>: The value must be an integer number between 1 and 127"));
    Serial.println(F("\t\t3) Example: to change I2C address of dissolved oxygen sensor => token_oxy and change_add165\n"));
    Serial.println(F("**** Control frame to scan the multiparameters probe ****"));
    Serial.println(F("\t\t- 'scan': to identify the AtlasScientific probes connected\n"));
    Serial.println(F("**** To ask a measure from any probe ****"));
    Serial.println(F("\t\t- 'ph': transmit a measure from the pH sensor"));
    Serial.println(F("\t\t- 'orp': transmit the captured luminosity in lux"));
    Serial.println(F("\t\t- 'lux': transmit a redox potential measure in mV"));
    Serial.println(F("\t\t- 'oxy': transmit a value of dissolved oxygen in mg/l"));
    Serial.println(F("\t\t- 'cond': transmit a measure of the conductivity in µS/cm"));
    Serial.println(F("\t\t- 'temp': transmit the temperature of the DS18B20 probe in °C\n"));
    
    Serial.println(F("**** To define a sampling rate ****"));
    Serial.println(F("\t* Seconds programmed can not be upper than 10 000."));
    Serial.println(F("\t* Minutes programmed can not be higher than 1000."));
    Serial.println(F("\t\t1) 'repeat?': to get the parameters state of the sampling rate"));
    Serial.println(F("\t\t2) 'repeat0': to stop the sampling of all probes"));
    Serial.println(F("\t\t3) 'repeat'<5s>: it represents a delay sampling of 5 seconds"));
    Serial.println(F("\t\t4) 'repeat'<20m>: it represents a delay sampling of 20 minutes\n"));
    
    Serial.println(F("**** To calibrate all connected sensors  ****"));
    Serial.println(F("\t* You can use 'cal?' command to get all instructions for the calibration\n"));
    Serial.println(F("**** Control frames to give the focus to a probe ****"));
    Serial.println(F("\t\t1) 'token?': to check which probe have got the focus"));
    Serial.println(F("\t\t2) 'token_'<probe>: with probe equivalent as ph, oxy, orp, cond, rtd or none"));
    Serial.println(F("\t\t3) Examples: dissolved oxygen => token_oxy, pH => token_ph, none => token_none"));  
    Divider(140, true, '=');
  }
}
/**********************************************************************************************************************************/
/* Function to extract an 16 bit integer in a string. If we know the name of the control command, we can define a position of the */
/* 16 bit integer. We have to transmit parameters to the function's arguments and Cde contains only litteral values.              */
/**********************************************************************************************************************************/
uint16_t detect_entier(char *ptr_mess, String Cde) {
  uint8_t h = 0;
  memset(TabASCII, Null, sizeof(TabASCII));           // void *memset(void *str, int c, size_t n)
  ptr_mess += Cde.length() * sizeof(char);              // to point the first digit after the command's name
  do {
    TabASCII[h++] = *(ptr_mess++);                      // we retrieve all the digit
  } while (*ptr_mess >= 0x30 && *ptr_mess <= 0x39);     // test on  next character
  return ConvASCIItoUint16(TabASCII);                   // as return atoi(TabASCII); but atoi() is out of order
}
/**********************************************************************************************************************************/
/* Function to extract a float number in a string. If we know the name of the control command, we can define a position of the    */
/* float number. We have to transmit parameters to the function's arguments and Cde contains only litteral values without the     */
/* digits of the float number. Precision of the float number depends of 4 bytes.                                                  */
/**********************************************************************************************************************************/
boolean detect_float(char *ptr_mess) {
  uint8_t k = 0;
  uint8_t NbrChar;
  float MyFloat;
  memset(TabASCII, Null, sizeof(TabASCII));                       // void *memset(void *str, int c, size_t n)
  NbrChar = GetNbrOfChar(ptr_mess);
  for (k = 0; k < NbrChar; k++) {
    if (isDigit(*ptr_mess) || (uint8_t)*ptr_mess == 0x2E) {       // ASCII char '.' = 0x2E
      TabASCII[k] = *(ptr_mess++);
    } else return false;
  }
  MyFloat = atof(TabASCII);
  //Serial.print(F("Float received: ")); Serial.println(MyFloat);
  if (MyFloat > zero) return true;
  else return false;
}
/**********************************************************************************************************************************/
/* Function to replace the standard method atoi(). The aim is to convert an ASCII representation of an integer number in value.   */
/* Normally we can not receive a lenght of string superior than 5 for a number strictly inferior than 65536.                      */
/**********************************************************************************************************************************/
uint16_t Convert_DecASCII_to_uint16(char *ptr_DecAscii) {
  uint32_t entier_verif;
  char *ptr_local;
  uint8_t p;
  uint8_t n = 0;
  ptr_local = ptr_DecAscii;
  do {
    if ((uint8_t)(*ptr_DecAscii) < 0x30 || (uint8_t)(*ptr_DecAscii) > 0x39) break;
    else {
      ptr_DecAscii++;
      n++;                                            // number of signficative characters
      //Serial.print(F("n = ")); Serial.println(n, DEC);
    }
  } while (1);
  if (n != 0) {
    ptr_local += (n - 1) * sizeof(char);              // the unit position in the array
    entier_verif = 0UL;
    for (p = 0; p < n; p++) {
      switch (p) {
        case 0:
          entier_verif += (uint32_t)(*(ptr_local--) - 0x30);
          break;
        case 1:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, 1));   // x 10
          break;
        case 2:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, 2));   // x 100
          break;
        case 3:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, 3));   // x 1000
          break;
        case 4:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, 4));   // x 10000
          break;
      }
    }
    if (entier_verif >= 65536) return 0;
    else return (uint16_t)entier_verif;
  }
  return 0;               // by default
}
/**********************************************************************************************************************************/
/* Function to replace the standard method atoi(). The aim is to convert an ASCII representation of an integer number in value.   */
/* Normally we can not receive a lenght of string superior than 10 for a number strictly inferior than 4 294 967 296.             */
/**********************************************************************************************************************************/
uint32_t Convert_DecASCII_to_uint32(char *ptr_AsciiUint32) {
  uint64_t entier_verif;
  char *ptr_local;
  uint8_t i;
  uint8_t p;
  uint8_t n = 0;
  ptr_local = ptr_AsciiUint32;
  
  do {
    if ((uint8_t)(*ptr_AsciiUint32) < 0x30 || (uint8_t)(*ptr_AsciiUint32) > 0x39) break;      // isDigit()
    else {
      ptr_AsciiUint32++;
      n++;                                            // number of signficative characters
    }
  } while (1);
  
  if (n != 0) {
    ptr_local += (n - 1) * sizeof(char);
    entier_verif = 0UL;
    //entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));           // this iteration is not valid
    //for (i = 0; i < n; i++) entier_verif += (int32_t)((*(ptr_local--) - 0x30) * pow(10, i));
    for (i = 0; i < n; i++) {
      switch (i) {
        case 0:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30));
          break;
        case 1:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 10
          break;
        case 2:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 100
          break;
        case 3:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 1 000
          break;
        case 4:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 10 000
          break;
        case 5:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 100 000
          break;
        case 6:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 1 000 000
          break;
        case 7:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 10 000 000
          break;
        case 8:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 100 000 000
          break;
        case 9:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 1 000 000 000
          break;
      }
    }
    if (entier_verif >= 4294967296) return 0;
    else return (uint32_t)entier_verif;
  }
  return 0UL;             // by default
}
/****************************************************************************************************/
/* Function to replace the useful sprintf for which the results on Arduino are not these awaited.   */
/* The conversion is simple from an hexadecimal value. We have to transform a value in its ASCII    */
/* form as a decimal value. sprintf do not function with long integer. The conversion begin with    */
/* the discover of the unit digit and she ends by the most significant digit of the value.          */
/* principle : modulo by 0x0A (first operation) => result have to be removed from initial value.    */
/* modulo =>          integer value strictly inferior at 0x0A, either the result between 0 and 9.   */
/* soustraction =>    we reduce the value by the result obtained by the modulo function.            */
/*                    (significative digit).                                                        */
/* division =>        fix the new result from which we will applied the modulo function.            */
/* EXAMPLE: value 0x0358 to convert                                                                 */
/*                    0x358 % A = 6 => 0x358 - 6 = 0x352 => 0x352 / A = 0x55                        */
/*                    0x55 % A = 5 => 0x55 - 5 = 0x50 => 0x50 / A = 0x8                             */
/*                    Result => 0x358 => '8' '5' '6'                                                */
/* The search is ended when the reuslt of the division by A give only one digit strictly inferior   */
/* than 0xA.                                                                                        */
/* The array which receive ASCII characters with a decimal representation has a dimension of        */
/* 20 characters. This array have to be public to be shared by all functions of this module.        */
/* ConvUintxx_tToAsciiChar[20];     // 2^64 = 18 446 744 073 709 551 616                            */
/****************************************************************************************************/
uint8_t ConvertUint16ToASCIIChar(char *ptrTAB, uint16_t valToConvert) {   // 5 possible characters 0 to 65,535
  char *ptrINIT;
  uint8_t k;
  uint8_t indice;
  uint8_t result_modulo;
  uint16_t result_division;
  ptrINIT = ptrTAB;
  for (k = 0; k < 20; k++) *(ptrTAB++) = Null;      // to initialize the array ConvUintxx_tToAsciiChar[]
  indice = 4;                                       // to locate the least significant digitof an unsigned integer uint18_t (65,535)
  ptrTAB = ptrINIT;
  ptrTAB += indice * sizeof(char);                  // we start with the least significant digit, the pointer will store the result
  do {
    result_modulo = (uint8_t)(valToConvert % 0x0A);
    *(ptrTAB--) = (char)(result_modulo + 0x30);
    indice--;
    result_division = (valToConvert - result_modulo) / 0x0A;
    valToConvert = result_division;
  } while (result_division > 9);                   // if result superior than 9 the search continue 
  *ptrTAB = (char)(0x30 + result_division);
  ptrTAB = ptrINIT;
  ptrINIT += indice * sizeof(char);                               // to retrieve the last location of the most sgnificant digit
  for (k = 0; k < 5 - indice; k++) *(ptrTAB++) = *(ptrINIT++);    // to retrieve an array which the first location is occuped
  *ptrTAB = Null;
  return 5 - indice;                                              // number of digits
}
/**********************************************************************************************************************************/
/* Function to convert an uint32_t (hexadecimal form) into a decimal ASCII representation to replace sprintf for long integer.    */
/* The conversion is made from low significant bit to high significant bit and the number of significant digit is returned.       */                
/**********************************************************************************************************************************/
uint8_t ConvertUint32ToASCIIChar(char *ptrTAB, uint32_t valToConvert) {   // 10 possible characters from 0 to 4,294,967,295
  char *ptrINIT;
  uint8_t k;
  uint8_t indice;
  uint8_t result_modulo;
  uint32_t result_division;
  ptrINIT = ptrTAB;
  for (k = 0; k < 20; k++) *(ptrTAB++) = Null;      // to initialize the array ConvUintxx_tToAsciiChar[]
  indice = 9;                                       // the low significant digit in the char array (unity)
  ptrTAB = ptrINIT;
  ptrTAB += indice * sizeof(char);                  // to fix the low digit
  do {
    result_modulo = (uint8_t)(valToConvert % 0x0A);
    *(ptrTAB--) = (char)(result_modulo + 0x30);     // ASCII char to display
    indice--;
    result_division = (valToConvert - result_modulo) / 0x0A;
    valToConvert = result_division;                 // new value for which we have to define the modulo
  } while (result_division > 9);                    // if result is between 0 and 15, we can identify all characters to be displayed
  *ptrTAB = (char)(0x30 + result_division);
  ptrTAB = ptrINIT;                                 // first position in the array
  ptrINIT += indice * sizeof(char);                 // to retrieve the last position of the most significant digit
  for (k = 0; k < 10 - indice; k++) *(ptrTAB++) = *(ptrINIT++);   // to retrieve an array starting with the fisrt position [0]
  *ptrTAB = Null;
  return 10 - indice;
}
/**********************************************************************************************************************************/
/* Function to display a measure from an Atlas Scientific probe selected by her address.                                          */
/* Return 0.0 if the probe is absent, any else value if present.                                                                  */
/**********************************************************************************************************************************/
float AtlasProbesMeasure(I2CAddresses_t AtlasDeviceAddress) {   // enum
  char *Myptr;
  float MyMeasure;
  uint8_t k;
  uint8_t Length;
  String LocalCde;
  String Answer;
  LocalCde.reserve(4);
  Answer.reserve(8);
  LocalCde = Cmd_Single_Read;                               // 'R'
  memset(StampCmd, Null, sizeof(StampCmd));                 // command sended to the stamp
  LocalCde.toCharArray(StampCmd, LocalCde.length() + 1);
  Length = LocalCde.length();
  I2C_call(StampCmd, AtlasDeviceAddress, Length);
  switch (AtlasDeviceAddress) {
    case DissolvedOxygen_Add:
      if (I2C_ProbesOfRaft.DO_Probe == true) I2C_call(StampCmd, AtlasDeviceAddress, Length);
      else return 0.0;
      break;
    case ORP_Add:
      if (I2C_ProbesOfRaft.ORP_Probe == true) I2C_call(StampCmd, AtlasDeviceAddress, Length);
      else return 0.0;
      break;
    case pH_Add:
      if (I2C_ProbesOfRaft.pH_Probe == true) I2C_call(StampCmd, AtlasDeviceAddress, Length);
      else return 0.0;
      break;
    case Conductivity_Add:
      if (I2C_ProbesOfRaft.EC_Probe == true) I2C_call(StampCmd, AtlasDeviceAddress, Length);
      else return 0.0;
      break;
    case RTD_Add:
      if (I2C_ProbesOfRaft.RTD_Probe == true) I2C_call(StampCmd, AtlasDeviceAddress, Length);
      else return 0.0;
      break;
  }
  Myptr = sensordata;                   // datas are stored in sensordata which is a shared array
  if (sensordata[0] == '1') {
    Answer = String(Myptr);
    Answer = Answer.substring(1);       // to remove the first character of the frame
    Answer.toCharArray(TabASCII, Answer.length() + 1);
    MyMeasure = atof(TabASCII);
  }
  #ifdef messagesON
    Divider(80, true, '*');
    Serial.print(F("Atlas Scientific probe measure is: ")); 
    Serial.print(MyMeasure);
    switch (AtlasDeviceAddress) {
      case DissolvedOxygen_Add:
        Serial.println(F(" mg/l"));
        break;
      case ORP_Add:
        Serial.println(F(" mV"));
        break;
      case pH_Add:
        Serial.println();
        break;
      case Conductivity_Add:
        Serial.println(F(" µS"));
        break;
      case RTD_Add:
        Serial.println(F(" °C"));
        break;
    }
    Divider(80, true, '*');
  #endif
  return MyMeasure;
}
/**********************************************************************************************************************************/
/* Function to change the I2C address of the stamp. The control command has this form: 'change_add'<105> (between 1 and 127)      */
/* It will necessary to check which probe has got the focus using the command 'token?' after a modification induced by other      */
/* commands like 'token_'<probe>                                                                                                  */
/**********************************************************************************************************************************/
void change_add_I2C(String Cde_received) {
  int InComingByte;               // virtual int read(void); method from HardwareSerial.h
  uint8_t New_Add_I2C;
  uint8_t NbrDigits;
  uint8_t NbrCdeChar;
  uint8_t k;
  String LocalCde;
  LocalCde.reserve(8);
  Cde_received = Cde_received.substring(10);
  Cde_received.toCharArray(TabASCII, Cde_received.length() + 1);
  New_Add_I2C = (uint8_t)Convert_DecASCII_to_uint16(&TabASCII[0]);
  Serial.print("changing address of the device into: ");
  Serial.print(New_Add_I2C, DEC);                 // http://www.mon-club-elec.fr/pmwiki_reference_arduino/pmwiki.php?n=Main.Serialprintln
  Serial.print(F(" (")); Serial.print(New_Add_I2C, HEX); Serial.print(F(")\n"));

  if (New_Add_I2C >= 1 and New_Add_I2C <= 127) {
    memset(TabASCII, Null, sizeof(TabASCII));     // void *memset(void *str, int c, size_t n)
    LocalCde = Chg_add_i2c;                                   // "I2C,"
    NbrCdeChar = LocalCde.length();
    LocalCde.toCharArray(TabASCII, NbrCdeChar + 1);           // to associate the additional character '\0'
    NbrDigits = ConvertUint16ToASCIIChar(&ConvUintxx_tToAsciiChar[0], (uint16_t)New_Add_I2C);
    for (k = 0; k < NbrDigits; k++) TabASCII[NbrCdeChar + k] = ConvUintxx_tToAsciiChar[k];
    NbrCdeChar += NbrDigits;
    CheckSetFocus(F("token?"));                               // show the probe which has the focus
    if (ProbeSelected != NoToken) {
      ShowFocus();
      Serial.println(F("A mandatory answer is awaited from you..."));
      Serial.print(F("Confirm the probe selection? (Y/N): "));
      /* the program waits a mandatory answer */
      memset(MyAnswer, Null, sizeof(MyAnswer));
      k = 0;
      InComingByte = Null;
      do {                                                    // polling loop
        if (Serial.available() != 0) {
          InComingByte = Serial.read();                       // virtual int read(void); from HardwareSerial.h due to larger datas from UART buffer
          MyAnswer[k++] = (char)InComingByte;
        }
      } while (InComingByte != '\n' || InComingByte != '\r'); // We wait the answer of the operator Yes or No
      MyAnswer[k] = Null;
      if (MyAnswer[0] == 'y' || MyAnswer[0] == 'Y') {
        I2C_call(&TabASCII[0], TokenAddressInProgress, NbrCdeChar);  // void I2C_call(char *Cde_busI2C, I2CAddresses_t ADDRESS, uint8_t nbr_carCde)
        Divider(80, true, '#');
        Serial.println(F("[I2C] The device restarts..."));
        check_i2c_connection(New_Add_I2C);
        Serial.print(F("[I2C] New I2C address applied: "));   // store string in flash memory
        Serial.print("0x");
        Serial.println(New_Add_I2C, HEX);                     // affichage de l'adresse du périphérique
        Divider(80, true, '#');
      } else {
        Serial.println(F("I2C address have not been changed..."));
        Serial.println(F("You must use the command 'token_'<probe> to select a probe."));
      }
    }
  } else Serial.println("The address is not an integer between 1 and 127");
}
/**********************************************************************************************************************************/
/* Function to check or to change which probe has got the focus. The command has the next form: 'token_'<probe>                   */
/* probe is the name of the probe using the acronyms: ph, oxy, orp, cond, rtd or none                                             */
/* If we want to check which probe has got the token, we need to use the command: 'token?'                                        */
/**********************************************************************************************************************************/
void CheckSetFocus(String Cde_received) {
  //char Probe[5];
  //char *ptr_probe;
  String ProbeToken;                  // the string which has to be compared
  ProbeToken.reserve(8);
  memset(TabASCII, Null, sizeof(TabASCII));
  Cde_received = Cde_received.substring(5);                       // to take off 'token'
  Cde_received.toCharArray(TabASCII, Cde_received.length() + 1);
  if (TabASCII[0] == '_') {
    ProbeToken = Cde_received.substring(1);                       // to take off '_'
    //memset(Probe, Nulle, sizeof(Probe));
    //ProbeToken.toCharArray(Probe, ProbeToken.length() + 1);
    //ptr_probe = Probe;
    if (ProbeToken.equals("ph")) {
      ProbeSelected = ph;
      Serial.println(F("Token is given to pH sensor"));
      return;
    } else if (ProbeToken.equals("oxy")) {
      ProbeSelected = oxy;
      Serial.println(F("Token is given to Dissolved Oxygen sensor"));
      return;
    } else if (ProbeToken.equals("orp")) {
      ProbeSelected = orp;
      Serial.println(F("Token is given to Oxydo Reduction Potential probe"));
      return;
    } else if (ProbeToken.equals("cond")) {
      ProbeSelected = cond;
      Serial.println(F("Token is given to Conductivity probe"));
      return;
    } else if (ProbeToken.equals("rtd")) {
      ProbeSelected = rtd;
      Serial.println(F("Token is given to Resistance Temperature Detector"));
      return;
    } else if (ProbeToken.equals("none")) {
      ProbeSelected = NoToken;
      Serial.println(F("No Token is given to any probe"));
      return;
    } else {
      Serial.println(F("[Error] Retype the command..."));
      return;
    }
  }
  if (TabASCII[0] == '?') ShowFocus();
}
/**********************************************************************************************************************************/
/* Function to show which probe has got the focus (or token) used for other commands causing changes.                             */
/**********************************************************************************************************************************/
void ShowFocus() {
  Serial.print(F("The probe selected is: "));
  switch (ProbeSelected) {                            // token_t ProbeSelected;
    case ph:
      Serial.println(F("pH sensor"));
      TokenAddressInProgress = pH_Add;                // I2CAddresses_t TokenAddressInProgress;
      break;
    case oxy:
      Serial.println(F("Dissolved Oxygen sensor"));
      TokenAddressInProgress = DissolvedOxygen_Add;
      break;
    case orp:
      Serial.println(F("Oxydo Reduction Potential probe"));
      TokenAddressInProgress = ORP_Add;
      break;
    case cond:
      Serial.println(F("Conductivity probe"));
      TokenAddressInProgress = Conductivity_Add;
      break;
    case rtd:
      Serial.println(F("Resistance Temperature Detector"));
      TokenAddressInProgress = RTD_Add;
      break;
    case NoToken:
      Serial.println(F("No token has been allocated..."));
      break;
  }
}
/**********************************************************************************************************************************/
/* During the calibrate operation, we can compensate the temperature of the solution in which the pH probe is.                    */
/* Temperature Compensation function has to be used when we calibrate pH and DO probes if the probe is present. We do not need to */
/* know the value of this temperature so it is an automatic call with the command (#define Compensate_temp "T,") <T,20.4>         */
/* When the operator defines himself the temperature using ASCII form, the command is 'comp'<DD.D>. To read the temperature used  */
/* for compensation, the command looks like: 'comp?'.                                                                             */
/**********************************************************************************************************************************/
void CompensatedTemp_pH_DO(String Cde_received) {           // if the operator wants to define a temperature with an other probe
  float TempMes;
  String LocalCde;
  uint8_t k;
  uint8_t CdeLength;
  LocalCde.reserve(10);
  Cde_received = Cde_received.substring(4);
  ShowFocus();
  Serial.println(F("Is it the probe you want to define the compensation temperature Y/N (y/n):"));
  FillMyAnswerArray();        // answer retrieve from the terminal
  if (MyAnswer[0] == 'y' || MyAnswer[0] == 'Y') {
    if (DS18B20IsPresent == true && Cde_received == "") {     // the value of the temperature from the DS18B20 probe is priority but we can send other values
      TempMes = MeasureTemp();
      memset(TabASCII, Null, sizeof(TabASCII));
      memset(StampCmd, Null, sizeof(StampCmd));               // command sended to the stamp
      LocalCde = Compensate_temp;                             // "T,"
      LocalCde.toCharArray(StampCmd, LocalCde.length() + 1);
      ConvFloatToString(TempMes, 1, &TabASCII[0]);    // void ConvFloatToString(float ConvertFloat, uint8_t NbrDecimals, char *DestArray) {...}
      k = 0;
      do {
        StampCmd[LocalCde.length() + k] = TabASCII[k];
      } while (TabASCII[k++] != Null);
      CdeLength = LocalCde.length() + k;
    } else if (DS18B20IsPresent == false && Cde_received == "") {
      Serial.println(F("The temperature probe is not present"));
      Serial.println(F("You need to transmit a temperature value using this format 'comp'<DD.D>"));
      return;     // without the call of the I2C control function
    } else {      // here Cde_received is not empty
      memset(TerminalCde, Null, sizeof(TerminalCde));
      Cde_received.toCharArray(TerminalCde, Cde_received.length() + 1);
      LocalCde = Compensate_temp;                             // "T,"
      LocalCde.toCharArray(StampCmd, LocalCde.length() + 1);
      if (TerminalCde[0] == '?') {
        StampCmd[LocalCde.length()] = '?';
        CdeLength = LocalCde.length() + 1;
      } else {
        k = 0;
        do {
          if (isDigit(TerminalCde[k]) || TerminalCde[k] == '.') StampCmd[LocalCde.length() + k] = TerminalCde[k];
          else return;
        } while (TerminalCde[++k] != Null);
        CdeLength = LocalCde.length() + k;
      }
    }
    I2C_call(&StampCmd[0], TokenAddressInProgress, CdeLength);                // the answer with a good aknowledgment is awaited
  } else {
    Serial.println(F("If you have to select an other probe,"));
    Serial.println(F("you can use the command 'token_'<probe> with the following acronyms:"));
    Serial.println(F("\t* oxy for Dissolved Oxygen sensor\n\t * ph for pH sensor\n\t* cond for Conductivity probe"));
    Serial.println(F("\t* orp for Oxydo Reduction Potential probe\n\t* rtd for Resistance Temperature Detector"));
    return;
  }
}
/**********************************************************************************************************************************/
/* Function to calibrate an Atlas Scientific probe among those availables. The operator has to choose a probe before the          */
/* calibration. So we need to know which probe has got the focus to applied calibration commands for which some ot them are       */
/* shared by several probes. All commands begin with the acronym "cal" and we can list command groups for each probe:             */
/* oxymeter: 'calzero' for measure with sodium bisulfite, 'calhigh' for a calibration with probe in contact with air (20.95 %).   */
/* conductivity:  'caldry' for a measure with probe out of solution,      'calDDDD' for a one point calibration,                  */
/*                'callowDDDDD' for a two points calibration,             'calhighDDDDDD' for a two points calibration            */
/* To get information the command is: 'cal?' and after this command we return to the main program to scrutinize other commands.   */
/* A focus have been initialized during the function call initI2C_Devices().                                                      */
/**********************************************************************************************************************************/
void Calibration(String Cde_received) {
  String LocalCde;
  String PartCde;
  uint8_t k;
  uint8_t NbrChar;
  float MyFloat;
  char *lcl_ptr;
  uint8_t NbrDigits;
  uint8_t CdeLength;
  uint16_t CalORP;
  uint16_t CalLowValue;
  uint32_t CalHighValue;
  LocalCde.reserve(10);
  PartCde.reserve(6);  
  Cde_received = Cde_received.substring(3);         // to delete the first 3 characters 'cal'
  memset(TerminalCde, Null, sizeof(TerminalCde));
  Cde_received.toCharArray(TerminalCde, Cde_received.length() + 1);

  if (Cde_received.equals("point")) return;         // to avoid all control commands which look like this function call
  
  if (TerminalCde[0] == '?') {
    Divider(140, true, '-');
    //Serial.println("\u03BC");   // µ char
    Serial.println(F("\t\t\t**** Control commands for the probes calibration ****\n"));
    Serial.println(F("\uFFED Conductivity probe:\t\t\tWARNING: dry calibration must always be done first."));
    Serial.println(F("\t\t\t\t\t1) 'caldry' to get the zero calibration with the probe out of water"));
    Serial.println(F("\t\t\t\t\t2) 'cal'<DDDD> to get a ONE point calibration with integer value lower than 5000"));
    Serial.println(F("\t\t\t\t\t3) 'callow'<DDDDD> for a TWO points calibration with integer value lower than 30000"));
    Serial.println(F("\t\t\t\t\t4) 'calhigh'<DDDDDD> for a TWO points calibration with integer value between 1000 and 100000\n"));
    Serial.println(F("\uFFED pH sensor:\t\t\t\tWARNING: issuing the 'calmid' command will clear all former calibration points."));
    Serial.println(F("\t\t\t\t\t1) 'calmid'<7.00> to get the middle point calibration with a reference solution as 7.00"));
    Serial.println(F("\t\t\t\t\t2) 'callow'<4.00> to get the low point calibration with a reference solution as 4.00"));
    Serial.println(F("\t\t\t\t\t3) 'calhigh'<10.00> to get the high point calibration with a reference solution as 10.00"));
    Serial.println(F("\t\t\t\t\t4) 'slope' to get informations about the slope of the probe\n"));
    Serial.println(F("\uFFED Dissolved Oxygen sensor:\t\tWARNING: The DO probe uses single or double point calibation."));
    Serial.println(F("\t\t\t\t\t2) 'calhigh' to calibrate the sensor to atmospheric oxygen level (20.95 %)"));
    Serial.println(F("\t\t\t\t\t2) 'calzero' to get the low point calibration with sodium bisulfite(0 %)\n"));
    Serial.println(F("\uFFED Oxydo Reduction Potential probe:\t1) 'cal'<DDD> to get only one point calibration using stanadard solution in mV\n"));
    Serial.println(F("\uFFED RTD probe:\t\t\t\tWARNING: RTD sensor uses single point calibration."));
    Serial.println(F("\t\t\t\t\t1) 'cal'<DDD.DD> to calibrate the temperature probe using any temperature measure considered as reference"));
    Divider(140, true, '-');
    return;
  }
  
  ShowFocus();              // fix the variable TokenAddressInProgress (I2CAddresses_t TokenAddressInProgress;)
  Serial.println(F("Do you want to change the probe selection?  Y/N (y/n):"));
  FillMyAnswerArray();      // answer retrieved from the terminal
  
  if (MyAnswer[0] == 'y' || MyAnswer[0] == 'Y') {
    Serial.println(F("You must use the command 'token_'<probe> with the following acronyms:"));
    Serial.println(F("\t* oxy for Dissolved Oxygen sensor\n\t* ph for pH sensor\n\t* cond for Conductivity probe"));
    Serial.println(F("\t* orp for Oxydo Reduction Potential probe\n\t* rtd for Resistance Temperature Detector"));
    Serial.println(F("Characters as ' and < or > have not to be used"));
    return;
  } else {
    switch (ProbeSelected) {                        // used in ShowFocus() function (token_t ProbeSelected;)
      case ph:
        if (isAlpha(TerminalCde[0]) == true) {
          if (TerminalCde[0] == 'm') {                            // 'calmid'<7.00>
            PartCde = Cde_received.substring(0, 3);               // to retrieve "mid"
            if (PartCde = "mid") {
              Cde_received = Cde_received.substring(3);           // to delete the next 3 characters 'mid'
              LocalCde = Cmd_Cal_mid;                             // "Cal,mid,"
              Serial.println(F("[WARNING] The midle calibration implies erazure of all calibration points...")); 
              Serial.println(F("[Calibration] You must place the pH sensor in the beaker filled with a 7.00 standard buffer..."));
            }
          } else if (TerminalCde[0] == 'h') {
            PartCde = Cde_received.substring(0, 4);               // to retrieve "high"
            if (PartCde = "high") {
              Cde_received = Cde_received.substring(4);           // to delete the next 3 characters 'mid'
              LocalCde = Cmd_Cal_high;                            // "Cal,high,"
              Serial.println(F("[WARNING] The high point calibration needs a long stabilization time..."));
              Serial.println(F("[Calibration] You must place the pH sensor in the beaker filled with a 10.00 standard buffer..."));
            }
          } else if (TerminalCde[0] == 'l') {
            PartCde = Cde_received.substring(0, 3);               // to retrieve "low"
            if (PartCde = "low") {
              Cde_received = Cde_received.substring(3);           // to delete the next 3 characters 'mid'
              LocalCde = Cmd_Cal_low;                             // "Cal,low,"
              Serial.println(F("[Calibration] You must place the pH sensor in the beaker filled with a 4.00 standard buffer..."));
            }
          } else {
            Serial.println(F("[ERROR] The calibration command does not fit anything knowed..."));
            return;
          }
          memset(FloatAscii, Null, sizeof(FloatAscii));
          Cde_received.toCharArray(FloatAscii, Cde_received.length() + 1);   
          if (detect_float(&FloatAscii[0])) {
            Serial.println(F("[Calibration] Is the pH sensor is immersed into the reference buffer? Y/N (y/n):"));
            FillMyAnswerArray();
            if (MyAnswer[0] == 'y') {
              memset(StampCmd, Null, sizeof(StampCmd));
              CdeLength = LocalCde.length();    //Serial.print(F("Length of CdeLength: ")); Serial.println(CdeLength, DEC); checked OK
              LocalCde.toCharArray(StampCmd, CdeLength + 1);    //Serial.print(F("last char: ")); Serial.println(StampCmd[CdeLength - 1]); checked OK
              Concatenate2Arrays(&StampCmd[0], &FloatAscii[0], &StampCmd[0]);
              //while (FloatAscii[k] != '\0') StampCmd[CdeLength + k] = FloatAscii[k++];            // checked OK => k = 4 when "4.02"
              NbrChar = GetNbrOfChar(&StampCmd[0]);
              Serial.print(F("[Calibration] Number of characters: ")); Serial.println(NbrChar, DEC);
              Serial.print(F("[Calibration] Stamp command : "));
              CdeLength = DisplayAsciiArray(&StampCmd[0]);                // here CdeLength = NbrChar
              I2C_call(&StampCmd[0], TokenAddressInProgress, CdeLength);  // TokenAddressInProgress update by ShowFocus() call
              Serial.println(F("[DEVICE] The control command has been sended"));
              MyFloat = pHMeasure(I2C_ProbesOfRaft.pH_Probe);
              Serial.print(F("[DEVICE] Checking pH measure: "));
              Serial.println(MyFloat);
            } else return;
          } else Serial.print(F("[Error] A float value has not been detected..."));
        }
        break;
      case oxy:
        if (isAlpha(TerminalCde[0]) == true) {
          if (TerminalCde[0] == 'h') {                            // 'calhigh'
            PartCde = Cde_received.substring(0, 4);               // to retrieve "high"
            if (PartCde = "high") {
              LocalCde = Cal_DO_Atmos;                            // "Cal"
              Serial.println(F("[Calibration] the probe needs to be calibrate to the atmosphéric oxygen level"));
              Serial.println(F("[WARNING] It will be better to take in account the atmospheric pressure value if depth is superior than 10 meters"));
              Serial.println(F("[Calibration] Is the DO sensor is out of a beaker? Y/N (y/n):"));
            }
          } else if (TerminalCde[0] == 'z') {
            PartCde = Cde_received.substring(0, 4);               // to retrieve "zero"
            if (PartCde = "zero") {
              LocalCde = Cal_DO_Zero;                             // "Cal,0"
              Serial.println(F("[Calibration] The zero point calibration is prepared using sodium sulfite (Na2SO3)"));
              Serial.println(F("[WARNING] It will be better to take in account the atmospheric pressure value if depth is superior than 10 meters"));
              Serial.println(F("[Calibration] Is the DO sensor is immersed into sodium sulfite buffer? Y/N (y/n):"));
            }
          } else {
            Serial.println(F("[ERROR] The calibration command does not fit anything knowed..."));
            return;
          }
          FillMyAnswerArray();
          if (MyAnswer[0] == 'y' || MyAnswer[0] == 'Y') {
            memset(StampCmd, Null, sizeof(StampCmd));
            CdeLength = LocalCde.length();
            LocalCde.toCharArray(StampCmd, CdeLength + 1); 
            I2C_call(&StampCmd[0], TokenAddressInProgress, CdeLength);  // TokenAddressInProgress update by ShowFocus() call  
          }
        }
        break;
      case orp:
        if (isDigit(TerminalCde[0]) == true) {
          memset(Uint16DecAscii, Null, sizeof(Uint16DecAscii));
          lcl_ptr = &Uint16DecAscii[0];
          k = 0;
          while (isDigit(TerminalCde[k])) *(lcl_ptr++) = TerminalCde[k++];
          *lcl_ptr = '\0';
          //Concatenate2Arrays(&StampCmd[0], &FloatAscii[0], &StampCmd[0]);
          CalORP = Convert_DecASCII_to_uint16(&Uint16DecAscii[0]);
          if (CalORP < 2000) {
            LocalCde = Cal_MySolution;                              // #define Cal_MySolution "Cal,"
            memset(StampCmd, Null, sizeof(StampCmd));
            CdeLength = LocalCde.length();
            LocalCde.toCharArray(StampCmd, CdeLength + 1);
            NbrChar = GetNbrOfChar(&Uint16DecAscii[0]);
            lcl_ptr = &Uint16DecAscii[0];
            for (k = 0; k < NbrChar; k++) StampCmd[CdeLength + k] = *(lcl_ptr++);
            I2C_call(&StampCmd[0], TokenAddressInProgress, CdeLength + NbrChar);
            //DisplayNbrCalPoints();
          }
        } else {
          Serial.println(F("[ERROR] The command looks like 'cal'<220> for a 220 mV reference potential electrode at 25 °C"));
          return;
        }
        break;
      case cond:
        if (isDigit(TerminalCde[0]) == true) {          // after remove "cal" prefix
          lcl_ptr = &Uint16DecAscii[0];                 // here the value is inferior than 5000
          k = 0;
          while (isDigit(TerminalCde[k])) *(lcl_ptr++) = TerminalCde[k++];
          *lcl_ptr = '\0';
          CalLowValue = Convert_DecASCII_to_uint16(&Uint16DecAscii[0]);
          NbrChar = GetNbrOfChar(&Uint16DecAscii[0]);
          // Only one point calibration
          k = 0;
          memset(Uint16DecAscii, Null, sizeof(Uint16DecAscii));
          //Concatenate2Arrays(&StampCmd[0], &FloatAscii[0], &StampCmd[0]);
          LocalCde = Cal_MySolution;                              // #define Cal_MySolution "Cal,"
          memset(StampCmd, Null, sizeof(StampCmd));
          CdeLength = LocalCde.length();
          LocalCde.toCharArray(StampCmd, CdeLength + 1);
          if (CalLowValue > 10 && CalLowValue <= 5000) {
            lcl_ptr = &Uint16DecAscii[0];
            for (k = 0; k < NbrChar; k++) StampCmd[CdeLength + k] = *(lcl_ptr++);
            StampCmd[CdeLength + NbrChar] = '\0';
            I2C_call(&StampCmd[0], TokenAddressInProgress, CdeLength + NbrChar);
            //DisplayNbrCalPoints();
          } else {
            Serial.println(F("You must use an other calibration solution between 10 µS and 5000 µS..."));
            return;
          }
        } else if (isAlpha(TerminalCde[0]) == true) {         // depends of the message either it is TWO points calibration or it is a dry calibration
          if (TerminalCde[0] == 'd') {
            if (Cde_received = "dry") {                       // after reduction it only stays "dry"
              memset(StampCmd, Null, sizeof(StampCmd));
              LocalCde = Cal_EC_Dry;                          // "Cal,dry"
              CdeLength = LocalCde.length();
              LocalCde.toCharArray(StampCmd, CdeLength + 1);
              I2C_call(&StampCmd[0], TokenAddressInProgress, CdeLength);
            }
          } else if (TerminalCde[0] == 'h') {
            PartCde = Cde_received.substring(0, 4);           // Cde_received.substring(from, to);
            if (PartCde = "high") {
              memset(Uint32DecAscii, Null, sizeof(Uint32DecAscii));
              Cde_received = Cde_received.substring(4);       // to delete "high"
              NbrDigits = Cde_received.length();
              memset(TerminalCde, Null, sizeof(TerminalCde));
              Cde_received.toCharArray(TerminalCde, NbrDigits + 1);
              lcl_ptr = &Uint32DecAscii[0];
              k = 0;
              while (isDigit(TerminalCde[k])) {
                *(lcl_ptr++) = TerminalCde[k++];
              }
              *lcl_ptr = '\0';
              CalHighValue = Convert_DecASCII_to_uint32(&Uint32DecAscii[0]);
              memset(StampCmd, Null, sizeof(StampCmd));
              LocalCde = Cmd_Cal_high;                                        // "Cal,high,"
              CdeLength = LocalCde.length();
              LocalCde.toCharArray(StampCmd, CdeLength + 1);                      
              if (CalHighValue >= 1000 && CalHighValue <= 100000UL) {
                NbrChar = Concatenate2Arrays(&StampCmd[0], &Uint32DecAscii[0], &StampCmd[0]);
                I2C_call(&StampCmd[0], TokenAddressInProgress, NbrChar);
              } else {
                Serial.println(F("The ASCII value defined in the command does not fit an appropriate value."));
                return;
              }
            }
          } else if (TerminalCde[0] == 'l') {
            PartCde = Cde_received.substring(0, 3);
            if (PartCde = "low") {
              memset(Uint16DecAscii, Null, sizeof(Uint16DecAscii));
              Cde_received = Cde_received.substring(3);                       // to delete "low"
              NbrDigits = Cde_received.length();
              memset(TerminalCde, Null, sizeof(TerminalCde));
              Cde_received.toCharArray(TerminalCde, NbrDigits + 1);
              lcl_ptr = &Uint16DecAscii[0];
              k = 0;
              while (isDigit(TerminalCde[k])) {
                *(lcl_ptr++) = TerminalCde[k++];
              }
              *lcl_ptr = '\0';
              CalLowValue = Convert_DecASCII_to_uint16(&Uint16DecAscii[0]);
              memset(StampCmd, Null, sizeof(StampCmd));
              LocalCde = Cmd_Cal_low;                                         // "Cal,low,"
              CdeLength = LocalCde.length();
              LocalCde.toCharArray(StampCmd, CdeLength + 1);
              if (CalLowValue <= 30000UL) {
                NbrChar = Concatenate2Arrays(&StampCmd[0], &Uint16DecAscii[0], &StampCmd[0]);
                I2C_call(&StampCmd[0], TokenAddressInProgress, NbrChar);
              } else {
                Serial.println(F("The ASCII value defined in the command does not fit an appropriate value."));
                return;
              }             
            }
          }
        }
        break;
      case rtd:                                                   // 'cal'<DDD.DD>
        if (isDigit(TerminalCde[0]) == true) {
          memset(FloatAscii, Null, sizeof(FloatAscii));
          Cde_received.toCharArray(FloatAscii, Cde_received.length() + 1);
          if (detect_float(&FloatAscii[0])) {
            LocalCde = Cal_MySolution;                            // "Cal,"
            memset(StampCmd, Null, sizeof(StampCmd));
            CdeLength = LocalCde.length();
            LocalCde.toCharArray(StampCmd, CdeLength + 1);
            k = 0;
            while (FloatAscii[k] != '\0') StampCmd[CdeLength + k] = FloatAscii[k++];
            CdeLength += k;
            I2C_call(&StampCmd[0], TokenAddressInProgress, CdeLength);          // TokenAddressInProgress update by ShowFocus() call
          }
        } else {
          Serial.println(F("[ERROR] The calibration command does not fit anything knowed..."));
          return;
        }
        break;
      case NoToken:
        Serial.println(F("No token has been allocated..."));
        Serial.println(F("You must select a probe using the command 'token'"));
        Serial.println(F("To find out all available commands with token control,"));
        Serial.println(F("you must use the help menu with 'help'command."));
        break;
    }
  }
}
/**********************************************************************************************************************************/
/* Function to eraze all datas of the calibration of an identified probe. True for all Atlas Scientific sensors.                  */
/**********************************************************************************************************************************/
void DeleteCalibration(String Cde_received) {
  String LocalCde;
  uint8_t CdeLength;
  LocalCde.reserve(10);
  ShowFocus();
  Serial.println(F("Do you want to clear the calibration data of the probe selection?  Y/N (y/n):"));
  FillMyAnswerArray();        // answer retrieve from the terminal
  if (MyAnswer[0] == 'y' || MyAnswer[0] == 'Y') {
    LocalCde = Cmd_CalClear;                      // #define Cmd_CalClear "Cal,clear"
    memset(StampCmd, Null, sizeof(StampCmd));
    CdeLength = LocalCde.length();
    LocalCde.toCharArray(StampCmd, CdeLength + 1);
    I2C_call(&StampCmd[0], TokenAddressInProgress, CdeLength);
    Serial.println(F("All calibration informations have been deleted..."));
    return;
  } else {
    Serial.println(F("You must change the focus to select an other probe using the command: 'token_'<probe>"));
    return;
  }
}
/**********************************************************************************************************************************/
/* Function to display the calibration state of each probe with the control command 'calpoint'. ASCII answer is '1' + "?Cal,D"    */
/* Normally we have to make the difference between the control command 'cal' for general purpose commands which calls the         */
/* functions Calibration() and the 'calpoint' command to check the number of calibation points.                                   */
/**********************************************************************************************************************************/
void DisplayNbrCalPoints(String Cde_received) {
  Cde_received = Cde_received.substring(3);           // to remove the text "cal"
  if (Cde_received.equals("point")) { 
    String LocalCde;
    uint8_t CdeLength;
    LocalCde.reserve(10);
    ShowFocus();
    Serial.println(F("Do you want to select an other probe?  Y/N (y/n):"));
    memset(MyAnswer, Null, sizeof(MyAnswer));
    FillMyAnswerArray();                              // answer retrieve from the terminal
    if (MyAnswer[0] == 'n' || MyAnswer[0] == 'N') {
      LocalCde = Cmd_NbrPtsCal;                       // #define Cmd_NbrPtsCal "Cal,?"
      memset(StampCmd, Null, sizeof(StampCmd));
      CdeLength = LocalCde.length();
      LocalCde.toCharArray(StampCmd, CdeLength + 1);
      I2C_call(&StampCmd[0], TokenAddressInProgress, CdeLength);
      Serial.println(F("The calibration has been done"));
      if (sensordata[0] == '1') {
        Serial.print(F("Number of calibrations points: "));
        Serial.println(sensordata[6]);                // same position for all devices
      }
    } else {
      Serial.println(F("You must change the focus to select an other probe using the command: 'token_'<probe>"));
      return;
    }
  } else Serial.println(F("The calibation command does not fit one of those knowed"));
}
/**********************************************************************************************************************************/
/* Function to initiate the I2C bus which is an object called Wire.                                                               */
/* MyI2CDevice is declared using a compilation directive #define MyI2CDevice Wire in this module.                                 */
/* ProbeSelected is an enumeration which select one probe among several when a measure is asked on the shared I2C bus.            */
/* Also TokenAddressInProgress content the I2C address in progress.                                                               */
/**********************************************************************************************************************************/
void initI2C_Devices(void) {
  MyI2CDevice.begin();                        // enable I2C port
  ProbeSelected = NoToken;                    // token_t ProbeSelected;
  TokenAddressInProgress = NoI2C_Add;         // I2CAddresses_t TokenAddressInProgress;
}
/**********************************************************************************************************************************/
/* Function to identify the address of the DS18B20 probe connected on the board using pin DS18B20Temp.                            */
/* This function is a mandatory call to inform the global variable DS18B20IsPresent of this module. This variable is shared by    */
/* many functions.                                                                                                                */
/**********************************************************************************************************************************/
boolean DallasTemperatureSearch(void) {
  uint8_t ADCresolution;
  DS18B20Probe.begin();             // Start up the library and check the presence of Probe on connection 10
  cmpt_5ms = 0;                     // timer of 5 ms
  do { } while(cmpt_5ms < 100);     // warmup of 500 ms
  if (DS18B20Probe.getAddress(&OneWireAddress[0], 0) == true) {       // OneWireAddress[] is a global array
    DS18B20IsPresent = true;        // global variable 
    ADCresolution = DS18B20Probe.getResolution(&OneWireAddress[0]);
    if (ADCresolution != 12) DS18B20Probe.setResolution(12);          // forces the 12 bits resolution
    Serial.println();
    Divider(80, true, '-');
    Serial.println(F("Probe DS18B20 is present."));
    AfficheAdresseCapteur();
  } else {
    Serial.println();
    Divider(80, true, '-');
    DS18B20IsPresent = false;
    Serial.println(F("\nTemperature probe DS18B20 is not connected or is faulty"));
  }
  Divider(80, true, '-');
  return DS18B20IsPresent;
}
/****************************************************************************************************/
/* Function which displays the content of the unidimensional array of the selected address probe.   */
/****************************************************************************************************/
void AfficheAdresseCapteur(void) {
  uint8_t *local_ptr;
  uint8_t k;
  local_ptr = &OneWireAddress[0]; 
  Serial.print(F("Selected probe address: "));
  for (k = 0; k < 8; k++) {
    if (*local_ptr > 9) Serial.print(*(local_ptr++), HEX);
    else {
      Serial.print('0');
      Serial.print(*(local_ptr++), DEC);
    }
    Serial.print(' ');
  }
  Serial.println();
}
/****************************************************************************************************/
/* Divider                                                                                          */
/****************************************************************************************************/
void Divider(uint8_t nbr_carac, boolean CRLFChar, char caract) {
  uint8_t i;
  for (i = 0; i < nbr_carac; i++) Serial.print(caract);
  //if (CRLFChar == true) Serial.println();
}
/**********************************************************************************************************************************/
/* Function to read the temperature of only one probe.                                                                            */
/* This module is informed of the probe presence with DS18B20IsPresent when subroutine DallasTemperatureSearch() is called.       */
/**********************************************************************************************************************************/
float MeasureTemp(void) {
  float result;
  Flags |= (1<<WatchdogDelayArmed);
  BusyTimeForWatchdog = BusyTimeForDS18B20;
  if (DS18B20IsPresent == true) {
    if (DS18B20Probe.requestTemperaturesByAddress(&OneWireAddress[0]) == true) {
      result = DS18B20Probe.getTempC(&OneWireAddress[0]);
    }
  } else return 0.0;
  Flags |= (1<<StopTheWatchdogTimer);
  return result;
}
/********************************************************************************************************/
/* Function to replace the equivalent method dtostrf.                                                   */
/* char *dtostrf(double val, signed char width, unsigned char prec, char *s)                            */
/* the aim is to fill the char array identified with the litteral values of a double or float number.   */
/* signed char width : number of alphanumeric values, comma included ('.').                             */
/* unsigned char prec : precision of the float or number of digits just after the comma.                */
/* essential link to get an example for conversion: https://www.esp8266.com/viewtopic.php?t=3592        */
/********************************************************************************************************/
void ConvFloatToString(float ConvertFloat, uint8_t NbrDecimals, char *DestArray) {
  uint8_t j, k;
  MyFloat_t LocalFloat;
  LocalFloat.value = ConvertFloat;
  uint32_t IEEE754Representation = 0;
  uint32_t IntegerResult;
  char Scratch_tab[10];       // to convert uint32_t in ASCII format
  uint8_t NbrChar;
  uint8_t CommaPosition;
  
  for (k = 4; k > 0; k--) {
    IEEE754Representation |= LocalFloat.byte_value[k - 1];
    if (k != 1) IEEE754Representation <<= 8;
  }
  if (IEEE754Representation & Sign_Mask) {        // #define Sign_Mask 0x80000000
    *(DestArray++) = '-';
    LocalFloat.value *= -1.0;
  }
  switch (NbrDecimals) {
    case 0:
      IntegerResult = (uint32_t)LocalFloat.value;
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);
      for (k = 0; k < NbrChar; k++) *(DestArray++) = Scratch_tab[k];
      *DestArray = Null;
      break;
    case 1:
      IntegerResult = (uint32_t)(LocalFloat.value * 10.0);                      // 23.8421 x 10 = 238.421 => 238
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);       // NbrChar = 3
      CommaPosition = NbrChar - 1;                                              // CommaPosition = 2
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      *(DestArray++) = Scratch_tab[CommaPosition];
      *DestArray = Null;
      break;
    case 2:
      IntegerResult = (uint32_t)(LocalFloat.value * 100.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);
      CommaPosition = NbrChar - 2;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 2; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;
    case 3:
      IntegerResult = (uint32_t)(LocalFloat.value * 1000.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);
      CommaPosition = NbrChar - 3;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 3; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;
    case 4:
      IntegerResult = (uint32_t)(LocalFloat.value * 10000.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);
      CommaPosition = NbrChar - 4;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 4; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;
    case 5:
      IntegerResult = (uint32_t)(LocalFloat.value * 100000.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);
      CommaPosition = NbrChar - 5;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 5; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;      
    default:
      IntegerResult = (uint32_t)(LocalFloat.value * 100.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);
      CommaPosition = NbrChar - 2;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 2; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;   
  }
}
/********************************************************************************************************/
/* Function to get an answer from operator using terminal. This function waits a mandatory answer from  */
/* the terminal. The short answer is 'Y' (Yes) or 'N' (No) but this function also would be useful to    */
/* retrieve other long response.                                                                        */
/********************************************************************************************************/
void FillMyAnswerArray(void) {
  uint8_t k = 0;
  int InComingByte = Null;
  memset(MyAnswer, Null, sizeof(MyAnswer));
  do {                                            // polling loop
    //Serial.print(F("InComingByte = ")); Serial.println(InComingByte, DEC);
    if (Serial.available() != 0) {
      InComingByte = Serial.read();               // virtual int read(void); from HardwareSerial.h due to larger datas from UART buffer
      MyAnswer[k++] = (char)InComingByte;
      if (InComingByte == '\n' || InComingByte == '\r') break;  // We wait the answer of the operator Yes or No
    }
  } while (1);
  MyAnswer[k] = Null;
  Serial.print(F("Your answer is: "));
  Serial.println(MyAnswer);                       // global array shared by all functions of this module
}
/********************************************************************************************************/
/* Function to convert an array filled with ASCII char which represent a decimal value.                 */
/********************************************************************************************************/
uint16_t ConvASCIItoUint16(char *ptr_tabl1) {
  uint8_t i, n;
  uint32_t IntegerValueSended;
  uint16_t result_local;
  char *ptr_local;
  ptr_local = ptr_tabl1;
  n = 0;        // to get the number of digit
  do {
    if ((uint8_t)(*ptr_tabl1) < 0x30 || (uint8_t)(*ptr_tabl1) > 0x39) break;  // get out from loop do while
    else {
      ptr_tabl1++;
      n++;
    }
  } while (1);
  if (n != 0) {
    ptr_local += (n - 1) * sizeof(char);  // to search the position of the unit
    IntegerValueSended = 0UL;
    for (i = 0; i < n; i++) {
      IntegerValueSended += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));
    }
    if (n >= 2) IntegerValueSended += (n - 2);
    #ifdef messagesON
      Serial.println(IntegerValueSended);
    #endif
  }
  if (IntegerValueSended >= 65536) return 0;
  else return (uint16_t)IntegerValueSended;
}
/**********************************************************************************************************************************/
/* Function to send measures to the terminal using a delay time. The aim of this function is to convert the ASCII integer included*/
/* in the command received and to activate the flag of the structure. Commands can operate on seconds and minutes with next       */
/* formats: 'repeat'<5s>, 'repeat'<20s>, 'repeat'<5m>, 'repeat'<250s>, 'repeat'<15m>, 'repeat'<3O0m>, 'repeat'<3543s> with the    */
/* following limits: seconds can not be superior than 10000 and also minutes can not be higher than 1000.                         */
/*                   seconds can not be lower than 5 seconds.                                                                     */
/* To request the sampling delay that has been programmed we use the command 'repeat?'                                            */
/* To stop the measures sampling, we use the command 'repeat0' which have effects to reset the integer values.                    */
/**********************************************************************************************************************************/
SamplingDelay_t SamplingDelayMeasure(String Cde_received, SamplingDelay_t LocalDelay) {
  uint8_t k;
  uint8_t NbrChar;
  uint16_t localResult;
  char *lcl_ptr;
  Cde_received = Cde_received.substring(6);               // string reduction to remove "repeat"
  memset(TabASCII, Null, sizeof(TabASCII));
  Cde_received.toCharArray(TabASCII, Cde_received.length() + 1);
  lcl_ptr = &TabASCII[0];
  memset(Uint16DecAscii, Null, sizeof(Uint16DecAscii));
  
  if (isAlpha(TabASCII[0]) == true) {                     // question mark '?' is not an alphabetical character ("repeat?")
    if (Cde_received.equals("zero")) {
      LocalDelay.RepeatedMeasures = false;
      LocalDelay.NbrSeconds = 0;
      LocalDelay.NbrMinutes = 0;
      LocalDelay.CompleteIntervalInSeconds = 0;
      Serial.println(F("The measure sampling has been ended."));
    }
  } else if (isDigit(TabASCII[0]) == true) {
    NbrChar = GetNbrOfChar(&TabASCII[0]);                                     // define the number of occupied cells  
    if (TabASCII[NbrChar - 1] == 's') {                                       // the last character
      for (k = 0; k < (NbrChar - 1); k++) Uint16DecAscii[k] = *(lcl_ptr++);   // before the 's' character
      localResult = Convert_DecASCII_to_uint16(&Uint16DecAscii[0]);
      if (localResult > 10000 || localResult < 5) {
        Serial.println(F("[Error] The delay in seconds can not be lower than 5... and"));
        Serial.println(F("[Error] also it can not be upper than 10 000..."));
        Serial.println(F("[WARNING] The sampling delay in secondes has not be changed..."));
        //if (LocalDelay.NbrMinutes == 0 && LocalDelay.NbrSeconds == 0) LocalDelay.RepeatedMeasures = false;
        //else LocalDelay.RepeatedMeasures = true;
      } else {
        LocalDelay.NbrSeconds = localResult;
        LocalDelay.RepeatedMeasures = true;
        LocalDelay.CompleteIntervalInSeconds = (LocalDelay.NbrMinutes * 60) + LocalDelay.NbrSeconds;
        //Serial.print(F("LocalDelay.NbrSeconds = ")); Serial.println(LocalDelay.NbrSeconds, DEC);
      }
    } else if (TabASCII[NbrChar - 1] == 'm') {                                       // the last character
      for (k = 0; k < (NbrChar - 1); k++) Uint16DecAscii[k] = *(lcl_ptr++);   // before the 'm' character
      localResult = Convert_DecASCII_to_uint16(&Uint16DecAscii[0]);
      //Serial.print(F("localResult = ")); Serial.println(localResult, DEC);
      if (localResult > 1000) {
        Serial.println(F("[Error] The delay in minutes can not be upper than 1000..."));
        Serial.println(F("[WARNING] The sampling delay in minutes has not be changed..."));
        //if (LocalDelay.NbrMinutes == 0 && LocalDelay.NbrSeconds == 0) LocalDelay.RepeatedMeasures = false;
        //else LocalDelay.RepeatedMeasures = true;
      } else {
        LocalDelay.NbrMinutes = localResult;
        LocalDelay.RepeatedMeasures = true;
        LocalDelay.CompleteIntervalInSeconds = (LocalDelay.NbrMinutes * 60) + LocalDelay.NbrSeconds;
      }
    }
  } else if (Cde_received.equals("?")) {
      Divider(140, true, '-');
      //Serial.println("\u03BC");   // µ char
      Serial.println(F("\t\t\t**** Control commands to define a sampling time ****\n"));
      Serial.print(F("\uFFED To change the number of seconds:\tWARNING: the ASCII value sended can not be lower than 5 "));
      //Serial.println(F("\t\t\t\t\t\t\tand seconds can not be upper than 10 000."));
      Serial.println(F("and also can not be upper than 10 000."));
      Serial.println(F("\t\t\t\t\t1) 'repeat'<15s> => to get 15 seconds delay time"));
      Serial.println(F("\t\t\t\t\t2) 'repeat'<125s> => to get 125 seconds delay time\n"));
      Serial.println(F("\uFFED To change the number of minutes:\tWARNING: the ASCII value sended can not be upper than 1000."));
      Serial.println(F("\t\t\t\t\t1) 'repeat'<10m> => to get 10 minutes delay time"));
      Serial.println(F("\t\t\t\t\t2) 'repeat'<580m> => to get 580 minutes delay time\n"));
      Serial.println(F("\uFFED To reset all stored values:\t\tWARNING: all values are erazed after the command below and the sampling will be stopped."));
      Serial.println(F("\t\t\t\t\t1) 'repeatzero' => to reset all stored values\n"));
      Divider(140, true, '-');
      Serial.print(F("The sampling delay in seconds is: "));
      Serial.println(LocalDelay.NbrSeconds, DEC);
      Serial.print(F("The sampling delay in minutes is: "));
      Serial.println(LocalDelay.NbrMinutes, DEC);
      Serial.print(F("Repeated measures is: "));
      if (LocalDelay.RepeatedMeasures == true) Serial.println(F("ON"));
      else Serial.println(F("OFF"));
      Divider(140, true, '-');
      //return LocalDelay;                        // the one which has been tansmitted as parameter
  }
  return LocalDelay;
}
/**********************************************************************************************************************************/
/* Global function to acquire measures from all probes. Several control commands allow to get measures and to edit the content    */
/* of each commands. In the text of the command we can use question mark '?' symbol to list commands content as 'meas?'           */
/* measall => send all measures from the avaliable sensors of the probe.                                                          */
/* The return type is a structure which contains all the measure from the available probes connected.                             */
/* DS18B20IsPresent is a global boolean variable which has been called when the DS18B20 is initialized.                           */
/**********************************************************************************************************************************/
ProbeMeasures_t Reading_probes(String Cde_received) {
  ProbeMeasures_t MyMeasures;       // contains all measures using float format for all buses
  char *lcl_ptr;
  String Answer;
  Answer.reserve(6);
  //InventoryProbes(I2C_ProbesOfRaft, DS18B20IsPresent);
  memset(TabASCII, Null, sizeof(TabASCII));
  Cde_received = Cde_received.substring(4);               // string reduction to remove "meas"
  Cde_received.toCharArray(TabASCII, Cde_received.length() + 1);
  lcl_ptr = TabASCII;
  Answer = String(lcl_ptr);
  if (TabASCII[0] == '?') {
    Divider(140, true, '-');
    Serial.println(F("\uFFED To get a measure from only one sensor, you can use these short version commands: "));
    Serial.println(F("\t- Dissolevd Oxygen\t\t => Cde = 'oxy'"));
    Serial.println(F("\t- pH sensor\t\t\t => Cde = 'ph'"));
    Serial.println(F("\t- RedOx potential\t\t => Cde = 'orp'"));
    Serial.println(F("\t- Conductivity\t\t\t => Cde = 'cond'"));
    Serial.println(F("\t- Temperature (DS18B20)\t\t => Cde = 'temp'"));
    Serial.println(F("\t- Luminosity\t\t\t => Cde = 'lux'"));
    Serial.println(F("\t- Temperature (RTD)\t\t => Cde = 'rtd'\n"));
    Serial.println(F("[WARNING] The values read using the commands above have not been stored anyway"));
    Serial.println(F("[WARNING] The command to get all measures and to store them from all available sensors is 'measall'"));
    Divider(140, true, '-');
  } else if (Answer.equals("all")) {
    if (I2C_ProbesOfRaft.pH_Probe == true) MyMeasures.pH_FloatValue = pHMeasure(I2C_ProbesOfRaft.pH_Probe);            // ProbeMeasures_t MyMeasures; (Global variable)
    if (I2C_ProbesOfRaft.DO_Probe == true) MyMeasures.DO_FloatValue = OxyMeasure(I2C_ProbesOfRaft.DO_Probe);           // I2CProbesConnected_t I2C_ProbesOfRaft; (Global variable)
    if (I2C_ProbesOfRaft.ORP_Probe == true) MyMeasures.ORP_FloatValue = orpMeasure(I2C_ProbesOfRaft.ORP_Probe);
    if (I2C_ProbesOfRaft.EC_Probe == true) MyMeasures.EC_FloatValue = ConductivityMeasure(I2C_ProbesOfRaft.EC_Probe);
    if (DS18B20IsPresent == true) MyMeasures.Temp_FloatValue = TempMeasure(DS18B20IsPresent);
    if (I2C_ProbesOfRaft.VEML7700_Probe == true) MyMeasures.Lux_FloatValue = luxmeter(I2C_ProbesOfRaft.VEML7700_Probe);
  }
  return MyMeasures;
}
/**********************************************************************************************************************************/
/* Global function to acquire measures from all probes. this function generate random values between boudaries.                   */
/**********************************************************************************************************************************/
ProbeMeasures_t RandomValues() {
  ProbeMeasures_t MyMeasures;       // contains all measures using float format for all buses
  MyMeasures.pH_FloatValue = (float)((float)(random(5000, 8000)) / 1000);
  MyMeasures.DO_FloatValue = (float)((float)(random(100, 10000)) / 1000);
  MyMeasures.ORP_FloatValue = (float)((float)(random(5000, 25000)) / 100);
  MyMeasures.EC_FloatValue = (float)((float)(random(20000, 1500000)) / 1000);
  MyMeasures.Temp_FloatValue = (float)((float)(random(1300, 2100)) / 100);
  MyMeasures.Lux_FloatValue = (float)((float)(random(1200, 80000)) / 100);
  return MyMeasures;
}
/**********************************************************************************************************************************/
/* Function to acquire and to display an Oxydo-reduction potential measure.                                                       */
/**********************************************************************************************************************************/
float orpMeasure(boolean ProbeReady) {
  char *Myptr;
  float MyMeasure = 0.0;
  uint8_t k;
  uint8_t Length;
  String AtlasCde;
  String Answer;
  AtlasCde.reserve(4);
  Answer.reserve(20); 
  if (ProbeReady == true) {
    digitalWrite(CdeNmosRFM0505, HIGH);                         // power on DC DC converter
    AtlasCde = Cmd_Single_Read;                                 // 'R'
    memset(StampCmd, Null, sizeof(StampCmd));                   // command sended to the stamp
    AtlasCde.toCharArray(StampCmd, AtlasCde.length() + 1);
    Length = AtlasCde.length();
    Serial.println(F("\t\uFFED Oxidation Reduction Potential sensor"));
    I2C_call(StampCmd, ORP_Add, Length);                        // we need to stop outputs on terminal
    Myptr = sensordata;                                         // datas are stored in sensordata array (global array)
    if (sensordata[0] == '1') {
      Answer = String(Myptr);
      Answer = Answer.substring(1);                             // to get only the ASCII float value
      Answer.toCharArray(TabASCII, Answer.length() + 1);        // global array [20]
      MyMeasure = atof(TabASCII);
    }
    SleepMode(ORP_Add);
    #ifdef messagesON
      Serial.print(F("\n\n\t\t\u2714 Oxidation-Reduction Potential measured: ")); 
      Serial.print(MyMeasure);
      Serial.println(F(" mV"));
      Divider(80, true, '*');
    #endif
    digitalWrite(CdeNmosRFM0505, LOW);                          // switch off DC DC converter
  }
  return MyMeasure;
}
/**********************************************************************************************************************************/
/* Function to acquire and to display a Conductivity measure.                                                                     */
/**********************************************************************************************************************************/
float ConductivityMeasure(boolean ProbeReady) {
  char *Myptr;
  float MyMeasure = 0.0;
  uint8_t k;
  uint8_t Length;
  String AtlasCde;
  String Answer;
  AtlasCde.reserve(4);
  Answer.reserve(20);
  if (ProbeReady == true) {
    digitalWrite(CdeNmosRFM0505, HIGH);                         // power on DC DC converter
    AtlasCde = Cmd_Single_Read;                                 // 'R'
    memset(StampCmd, Null, sizeof(StampCmd));                   // command sended to the stamp
    AtlasCde.toCharArray(StampCmd, AtlasCde.length() + 1);
    Length = AtlasCde.length();
    Serial.println(F("\t\uFFED Conductivity sensor"));
    I2C_call(StampCmd, Conductivity_Add, Length);
    Myptr = sensordata;                                         // datas are stored in sensordata array
    if (sensordata[0] == '1') {
      Answer = String(Myptr);
      Answer = Answer.substring(1);
      Answer.toCharArray(TabASCII, Answer.length() + 1);
      MyMeasure = atof(TabASCII);
    }
    SleepMode(Conductivity_Add);
    #ifdef messagesON
      Serial.print(F("\n\n\t\t\u2714 Conductivity measured: "));
      Serial.print(MyMeasure);
      Serial.println(F(" µS"));
      Divider(80, true, '*');
    #endif
    digitalWrite(CdeNmosRFM0505, LOW);                          // switch off DC DC converter
  }
  return MyMeasure;
}
/**********************************************************************************************************************************/
/* Function to acquire and to display a pH measure.                                                                               */
/**********************************************************************************************************************************/
float pHMeasure(boolean ProbeReady) {
  char *Myptr;
  float MyMeasure = 0.0;
  uint8_t k;
  uint8_t Length;
  String AtlasCde;
  String Answer;
  AtlasCde.reserve(4);
  Answer.reserve(20);
  if (ProbeReady == true) {
    digitalWrite(CdeNmosRFM0505, HIGH);                         // power on DC DC converter
    AtlasCde = Cmd_Single_Read;                                 // 'R'
    memset(StampCmd, Null, sizeof(StampCmd));                   // command sended to the stamp
    AtlasCde.toCharArray(StampCmd, AtlasCde.length() + 1);
    Length = AtlasCde.length();
    Serial.println(F("\t\uFFED pH sensor"));
    I2C_call(StampCmd, pH_Add, Length);
    Myptr = sensordata;                                         // datas are stored in sensordata array
    if (sensordata[0] == '1') {
      Answer = String(Myptr);
      Answer = Answer.substring(1);
      Answer.toCharArray(TabASCII, Answer.length() + 1);
      MyMeasure = atof(TabASCII);
    }
    SleepMode(pH_Add);
    #ifdef messagesON
      Serial.print(F("\n\n\t\t\u2714 pH measured: "));
      Serial.println(MyMeasure);
      Divider(80, true, '*');
    #endif
    digitalWrite(CdeNmosRFM0505, LOW);                          // switch off DC DC converter
  }
  return MyMeasure;
}
/**********************************************************************************************************************************/
/* Function to acquire and to display an oxygen measure.                                                                          */
/**********************************************************************************************************************************/
float OxyMeasure(boolean ProbeReady) {
  char *Myptr;
  float MyMeasure = 0.0;
  uint8_t k;
  uint8_t Length;
  String AtlasCde;
  String Answer;
  AtlasCde.reserve(4);
  Answer.reserve(20);
  if (ProbeReady == true) {
    digitalWrite(CdeNmosRFM0505, HIGH);                         // power on DC DC converter
    AtlasCde = Cmd_Single_Read;                                 // 'R'
    memset(StampCmd, Null, sizeof(StampCmd));                   // command sended to the stamp
    AtlasCde.toCharArray(StampCmd, AtlasCde.length() + 1);
    Length = AtlasCde.length();
    Serial.println(F("\t\uFFED Dissolved Oxygen sensor"));
    I2C_call(StampCmd, DissolvedOxygen_Add, Length);
    Myptr = sensordata;                                   // datas are stored in sensordata array
    if (sensordata[0] == '1') {
      Answer = String(Myptr);
      Answer = Answer.substring(1);
      Answer.toCharArray(TabASCII, Answer.length() + 1);
      MyMeasure = atof(TabASCII);
    }
    SleepMode(DissolvedOxygen_Add);
    #ifdef messagesON
      Serial.print(F("\n\n\t\t\u2714 Dissolved-oxygen measured: "));
      Serial.print(MyMeasure);
      Serial.println(F(" mg/l"));
      Divider(80, true, '*');
    #endif
    digitalWrite(CdeNmosRFM0505, LOW);                          // switch off DC DC converter
  }
  return MyMeasure;
}
/**********************************************************************************************************************************/
/* Function to acquire and to display a temperature measure.                                                                      */
/* The probe is connected on the OneWire bus and so do not share the I2C bus of Atlas Scientific probes.                          */
/**********************************************************************************************************************************/
float TempMeasure(boolean ProbeReady) {
  float MyMeasure = 0.0;
  uint8_t k;
  if (ProbeReady == true) {                             // DS18B20
    MyMeasure = MeasureTemp();
    #ifdef messagesON
      Serial.println(F("\t\uFFED DS18B20 sensor"));
      Serial.print(F("\n\t\t\u2714 Temperature measured: ")); 
      Serial.print(MyMeasure);
      Serial.println(F(" °C"));
      Divider(80, true, '*');
    #endif
  } else Serial.println(F("Probe is not connected or is faulty..."));
  return MyMeasure;
}
/**********************************************************************************************************************************/
/* Function to get the number of character in an array. Stopped until it detects the Null character '\0'.                         */
/* Function has been checked several times.                                                                                       */
/**********************************************************************************************************************************/
uint8_t GetNbrOfChar(char *ptr_lcl) {
  uint8_t n = 0;
  do {
    n++;
  } while (*(ptr_lcl++) != '\0');
  n -= 1;
  return n;
}
/**********************************************************************************************************************************/
/* Function to display on terminal the content of a char array. Displaying is stopped until it detects the Null character '\0'.   */
/* The array is occupied by at least one character.                                                                               */
/**********************************************************************************************************************************/
uint8_t DisplayAsciiArray(char *ptr_lcl) {
  uint8_t n = 0;
  do {
    Serial.print(*(ptr_lcl++));
    n++;
  } while (*ptr_lcl != '\0');
  Serial.println();
  return n;
}
/**********************************************************************************************************************************/
/* Function to concatenate two ascii arrays into a string format. The lenght of the string can not exceed 32 characters.          */
/* This function returns the number of characters contained ine the final array.                                                  */
/**********************************************************************************************************************************/
uint8_t Concatenate2Arrays(char *ptr_FirstArray, char *ptr_SecondArray, char *ptr_FinalArray) {
  uint8_t NbrCharFirstArray;
  uint8_t NbrCharSecondArray;
  uint8_t k;
  memset(ConvStringAsArray, Null, sizeof(ConvStringAsArray));           // char ConvStringAsArray[32];
  NbrCharFirstArray = GetNbrOfChar(ptr_FirstArray);
  for (k = 0; k < NbrCharFirstArray; k++) ConvStringAsArray[k] = *(ptr_FirstArray++);
  NbrCharSecondArray = GetNbrOfChar(ptr_SecondArray);
  for (k = 0; k < NbrCharSecondArray; k++) ConvStringAsArray[k + NbrCharFirstArray] = *(ptr_SecondArray++);
  k = 0;
  while (ConvStringAsArray[k] != '\0') *(ptr_FinalArray++) = ConvStringAsArray[k++];
  *ptr_FinalArray = '\0';
  return (NbrCharFirstArray + NbrCharSecondArray);
}
/**********************************************************************************************************************************/
/* Function to make an inventory of all connected probes identified by the main program.                                          */
/* MyProbes is a local variable which can be informed by the global variable I2C_ProbesOfRaft.                                  */
/**********************************************************************************************************************************/
void InventoryProbes(I2CProbesConnected_t MyProbes, boolean DS18B20Probe) {
  String LocalCde;
  uint8_t NbrChar;
  uint8_t StarsNbr;
  LocalCde.reserve(20);
  LocalCde = InventoryText;
  LocalCde.toCharArray(TabASCII, LocalCde.length() + 1);
  NbrChar = GetNbrOfChar(&TabASCII[0]);
  StarsNbr = 80 - (NbrChar + 2);
  Divider((uint8_t)(StarsNbr / 2), false, '*'); Serial.print(' ');
  Serial.print(&TabASCII[0]); Serial.print(' ');
  Divider((StarsNbr / 2), true, '*');
  if (MyProbes.DO_Probe == true) Serial.println(F("[Dissolved Oxygen sensor (I2C)]\t\t\t => present"));
  else Serial.println(F("[Dissolved Oxygen sensor (I2C)]\t\t\t => not present or is faulty"));
  if (MyProbes.pH_Probe == true) Serial.println(F("[pH probe (I2C)]\t\t\t\t => present"));
  else Serial.println(F("[pH probe (I2C)]\t\t\t\t => not present or is faulty"));
  if (MyProbes.ORP_Probe == true) Serial.println(F("[Oxydo Reduction Potential sensor (I2C)]\t => present"));
  else Serial.println(F("[Oxydo Reduction Potential sensor (I2C)]\t => not present or is faulty"));
  if (MyProbes.EC_Probe == true) Serial.println(F("[Conductivity probe (I2C)]\t\t\t => present"));
  else Serial.println(F("[Conductivity probe (I2C)]\t\t\t => not present or is faulty"));
  if (MyProbes.RTD_Probe == true) Serial.println(F("[RTD sensor (I2C)]\t\t\t\t => present"));
  else Serial.println(F("[RTD sensor (I2C)]\t\t\t\t => not present or is faulty"));
  if (MyProbes.VEML7700_Probe == true) Serial.println(F("[Light sensor (I2C)]\t\t\t\t => present"));
  else Serial.println(F("[Light sensor (I2C)]\t\t\t\t => not present or is faulty"));
  if (DS18B20Probe == true) Serial.println(F("[Temperature sensor (OneWire)]\t\t\t => present"));
  else Serial.println(F("[Temperature sensor (OneWire)]\t\t\t => not present or is faulty"));
  Divider(80, true, '*');
}
/* **************************************************************************************************************************** */
/* SLEEP command to enter sleep mode and low power.                                                                             */
/* **************************************************************************************************************************** */
void SleepMode(I2CAddresses_t ADDRESS) {
  String AtlasCde;
  AtlasCde.reserve(6);
  uint8_t k;
  char *Cde_busI2C;
  uint8_t Length;
  
  AtlasCde = Cmd_SLEEP;
  Length = AtlasCde.length();
  memset(StampCmd, Null, sizeof(StampCmd));             // command sended to the stamp global array [32]
  AtlasCde.toCharArray(StampCmd, AtlasCde.length() + 1);
  Cde_busI2C = &StampCmd[0];
  MyI2CDevice.beginTransmission((uint8_t)ADDRESS);      // call the circuit by its ID number : void TwoWire::beginTransmission(uint8_t address) https://www.arduino.cc/en/Reference/WireBeginTransmission
  MyI2CDevice.write(Cde_busI2C, Length);                // virtual size_t write(const uint8_t *, size_t); => here is the real transmission of a control command to the Atlas Scietific stamp
  MyI2CDevice.endTransmission();
  #ifdef messagesON
    Serial.print(F("Control command transmitted to the stamp: "));
    for (k = 0; k < Length; k++) Serial.print(*(Cde_busI2C++));
  #endif                 
}
/* **************************************************************************************************************************** */
/* Function which increment time every second each time she is called.                                                          */
/* the returned type is complex to allow the transfer of boolean and integer values.                                            */
/* **************************************************************************************************************************** */
ElapsedTime_t IncrementMyGSMtime(ElapsedTime_t MyTime) {
  MyTime.seconds++;
  if (MyTime.seconds == 60) {
    MyTime.seconds = 0;
    MyTime.minutes++;
    if (MyTime.minutes == NbrMinutesToResetGSM) {           // #define NbrMinutesToResetGSM 60
      MyTime.minutes = 0;
      MyTime.ResetGSM = true;
    }
  }
  return MyTime;
}
/********************************************************************************************************/
/* Function to display the content of an array until it encounters a Null character.                    */
/********************************************************************************************************/
void DisplayArrayContentFunctions(char *lclptr, boolean CRLF) {
  do {
    Serial.print(*(lclptr++));
  } while (*lclptr != Null);
  if (CRLF == true) Serial.println();
}
/****************************************************************************************************/  
/* identification of probes on I2C bus from which instance are declared.                            */
/* With power supply 5 V: ads1115.setGain(GAIN_ONE);  // 1x gain +/- 4.096V 1 bit = 2mV             */
/****************************************************************************************************/
void ADCStart(adsGain_t ADC_Gain) {
  MyADC.begin(0x48);                  // (ADDR tied to GND)
  MyADC.setGain(ADC_Gain);            // 2x gain +/- 2.048 Volts LSB = 62,5 µV
  MyADC.setDataRate(ADS1X15_REG_CONFIG_MODE_SINGLE);
}
/**************************************************************************************************************************/
/* Function to read all the voltage values applied on the ADC.                                                            */
/* GAIN ONE applied: +/- 4.096 volts as reference voltage                                                                 */
/* 1) Battery between 3 volts and 4.2 volts, so with gain 1 and resistive bridge of 1/2 (2 x 47k): measure x 2            */
/* 2) Voltage on MPPToutput is 5 volts, so with gain 1 and resistive bridge of 1/2 (2 x 47k): measure x 2                 */
/* 3) SolarPanel has a potential which can climb to 18 volts so with gain 1 and resistive bridge of 1/11 (20k + 200k):    */
/* => measure x 11 to get the true voltage at the terminals of the photovoltaic panel.                                    */
/* 4) Batteries mounted as 3 elements (3s) provide a max voltage of 12,6 volts so resistive bridge is 1/4:                */
/* => resistors as 200k and 75k allow to get a divisor by 3.6666. So with this resistive bridge, we can measure a maximum */
/* volatge of 3.43 volts which is lower than the reference voltage of 4.096V.
/* 5) A photovoltaic panel with Voc = 21.8 V allows to get a  */
/* solar panel with Open-Circuit Voltage = from 21.8 to 22.4 volts
/* Optimum Operating Current (current maximum) and Optmum Operating Voltage (voltage at maximum irradiance
 */
/**************************************************************************************************************************/
Voltages_t AcquireVoltageValues() {
  Voltages_t MyVoltages;
  int16_t adc;
  adc = MyADC.readADC_Differential_0_1();                               // ddp_bat
  MyVoltages.ddp_bat = MyADC.computeVolts(adc) * 2;
  Serial.println(F("\t\uFFED ADS1115 analog converter for batteries (Lithium-Ion) and Photovoltaic modules"));
  Serial.print(F("\n\t\t\u2714 Battery voltage: "));
  Serial.print(MyVoltages.ddp_bat); Serial.println(F(" volts"));
  adc = MyADC.readADC_Differential_2_3();                               // SolarPanel
  MyVoltages.ddp_panel = MyADC.computeVolts(adc) * 11;
  Serial.print(F("\n\t\t\u2714 Solar panel voltage: "));
  Serial.print(MyVoltages.ddp_panel); Serial.println(F(" volts"));
  return MyVoltages;
}
/********************************************************************************************************/
/* Function to configure the direction and the default state of each GPIO used for control all the Load */
/* Switches. By default, all GPIO are configured with high level to power on all devices.               */
/* MyPowerSwitches is a global variable declared in this module. PowerSwitches_t MyPowerSwitches;       */
/********************************************************************************************************/
void GPIOConfigurationAndPowerON() {
  pinMode(GPIO_pH_Switch, OUTPUT);                  // first the +5 V to power the device
  digitalWrite(GPIO_pH_Switch, HIGH);
  pinMode(GPIO_pH_isolator, OUTPUT);
  digitalWrite(GPIO_pH_isolator, HIGH);
  MyPowerSwitches.pHProbePowered = true;            // PowerSwitches_t MyPowerSwitches;
  pinMode(GPIO_ORP_Switch, OUTPUT);                 // first the +5 V to power the device
  digitalWrite(GPIO_ORP_Switch, HIGH);
  pinMode(GPIO_ORP_isolator, OUTPUT);
  digitalWrite(GPIO_ORP_isolator, HIGH);
  MyPowerSwitches.ORPProbePowered = true;
  pinMode(GPIO_EC_Switch, OUTPUT);                  // first the +5 V to power the device
  digitalWrite(GPIO_EC_Switch, HIGH);
  pinMode(GPIO_EC_isolator, OUTPUT);
  digitalWrite(GPIO_EC_isolator, HIGH);
  MyPowerSwitches.ECProbePowered = true;
  pinMode(GPIO_RTD_Switch, OUTPUT);                 // first the +5 V to power the device
  digitalWrite(GPIO_RTD_Switch, HIGH);
  pinMode(GPIO_RTD_isolator, OUTPUT);
  digitalWrite(GPIO_RTD_isolator, HIGH);
  MyPowerSwitches.RTDProbePowered = true;
  pinMode(GPIO_DO_Switch, OUTPUT);                  // first the +5 V to power the device
  digitalWrite(GPIO_DO_Switch, HIGH);
  pinMode(GPIO_DO_isolator, OUTPUT);
  digitalWrite(GPIO_DO_isolator, HIGH);
  MyPowerSwitches.DOProbePowered = true;
  pinMode(GPIO_DS18_Switch, OUTPUT);                // first the +5 V to power the device
  digitalWrite(GPIO_DS18_Switch, HIGH);
  pinMode(GPIO_DS18_isolator, OUTPUT);
  digitalWrite(GPIO_DS18_isolator, HIGH);
  MyPowerSwitches.DS18B20Powered = true;
  pinMode(GPIO_VEML_Switch, OUTPUT);                // first the +5 V to power the device
  digitalWrite(GPIO_VEML_Switch, HIGH);
  pinMode(GPIO_VEML_isolator, OUTPUT);
  digitalWrite(GPIO_VEML_isolator, HIGH);
  MyPowerSwitches.VEML7700Powered = true;
  pinMode(CdeNmosRFM0505, OUTPUT);
  digitalWrite(CdeNmosRFM0505, LOW);                // to switch off the power supply ISO-VCC from the DC DC converter RFM-0505
}
/********************************************************************************************************/
/* Function to stop power supply for all devices using load switches and the OFF pin of each isolator.  */
/********************************************************************************************************/
void IsolatorAndPowerOFF() {
  digitalWrite(GPIO_pH_Switch, LOW);
  digitalWrite(GPIO_pH_isolator, LOW);
  MyPowerSwitches.pHProbePowered = false;
  digitalWrite(GPIO_ORP_Switch, LOW);
  digitalWrite(GPIO_ORP_isolator, LOW);
  MyPowerSwitches.ORPProbePowered = false;
  digitalWrite(GPIO_EC_Switch, LOW);
  digitalWrite(GPIO_EC_isolator, LOW);
  MyPowerSwitches.ECProbePowered = false;
  digitalWrite(GPIO_RTD_Switch, LOW);
  digitalWrite(GPIO_RTD_isolator, LOW);
  MyPowerSwitches.RTDProbePowered = false;
  digitalWrite(GPIO_DO_Switch, LOW);
  digitalWrite(GPIO_DO_isolator, LOW);
  MyPowerSwitches.DOProbePowered = false;
  digitalWrite(GPIO_DS18_Switch, LOW);
  digitalWrite(GPIO_DS18_isolator, LOW);
  MyPowerSwitches.DS18B20Powered = false;
  digitalWrite(GPIO_VEML_Switch, LOW);
  digitalWrite(GPIO_VEML_isolator, LOW);
  MyPowerSwitches.VEML7700Powered = false;
}
/********************************************************************************************************/
/* Function to activate or to inhibit the power supply of each devices using load switch.               */
/* This function has to modify the global variable which is PowerSwitches_t MyPowerSwitches;            */
/********************************************************************************************************/
void PowerSupplyForDevices(PowerSwitches_t LocalSwitches) {  
  switch (LocalSwitches.TheSelectedDevice) {
    case AllProbes:
      if (LocalSwitches.pHProbePowered == true) {
        digitalWrite(GPIO_pH_Switch, HIGH);                   // power ON
        digitalWrite(GPIO_pH_isolator, HIGH);                 // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_pH_Switch, LOW);                    // power OFF
        digitalWrite(GPIO_pH_isolator, LOW);                  // ISO-VCC not applied to EZO circuit        
      }
      if (LocalSwitches.ORPProbePowered == true) {
        digitalWrite(GPIO_ORP_Switch, HIGH);                  // power ON
        digitalWrite(GPIO_ORP_isolator, HIGH);                // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_ORP_Switch, LOW);                   // power OFF
        digitalWrite(GPIO_ORP_isolator, LOW);                 // ISO-VCC not applied to EZO circuit        
      }
      if (LocalSwitches.ECProbePowered == true) {
        digitalWrite(GPIO_EC_Switch, HIGH);                   // power ON
        digitalWrite(GPIO_EC_isolator, HIGH);                 // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_EC_Switch, LOW);                    // power OFF
        digitalWrite(GPIO_EC_isolator, LOW);                  // ISO-VCC not applied to EZO circuit        
      }
      if (LocalSwitches.RTDProbePowered == true) {
        digitalWrite(GPIO_RTD_Switch, HIGH);                  // power ON
        digitalWrite(GPIO_RTD_isolator, HIGH);                // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_RTD_Switch, LOW);                   // power OFF
        digitalWrite(GPIO_RTD_isolator, LOW);                 // ISO-VCC not applied to EZO circuit        
      }
      if (LocalSwitches.DOProbePowered == true) {
        digitalWrite(GPIO_DO_Switch, HIGH);                   // power ON
        digitalWrite(GPIO_DO_isolator, HIGH);                 // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_DO_Switch, LOW);                    // power OFF
        digitalWrite(GPIO_DO_isolator, LOW);                  // ISO-VCC not applied to EZO circuit        
      }
      if (LocalSwitches.VEML7700Powered == true) {
        digitalWrite(GPIO_VEML_Switch, HIGH);                 // power ON
        digitalWrite(GPIO_VEML_isolator, HIGH);               // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_VEML_Switch, LOW);                  // power OFF
        digitalWrite(GPIO_VEML_isolator, LOW);                // ISO-VCC not applied to EZO circuit        
      }
      if (LocalSwitches.DS18B20Powered == true) {
        digitalWrite(GPIO_DS18_Switch, HIGH);                 // power ON
        digitalWrite(GPIO_DS18_isolator, HIGH);               // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_DS18_Switch, LOW);                  // power OFF
        digitalWrite(GPIO_DS18_isolator, LOW);                // ISO-VCC not applied to EZO circuit        
      }
      break;
    case pHProbe:
      if (LocalSwitches.pHProbePowered == true) {
        digitalWrite(GPIO_pH_Switch, HIGH);                   // power ON
        digitalWrite(GPIO_pH_isolator, HIGH);                 // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_pH_Switch, LOW);                    // power OFF
        digitalWrite(GPIO_pH_isolator, LOW);                  // ISO-VCC not applied to EZO circuit        
      }
      break;
    case ORPProbe:
      if (LocalSwitches.ORPProbePowered == true) {
        digitalWrite(GPIO_ORP_Switch, HIGH);                  // power ON
        digitalWrite(GPIO_ORP_isolator, HIGH);                // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_ORP_Switch, LOW);                   // power OFF
        digitalWrite(GPIO_ORP_isolator, LOW);                 // ISO-VCC not applied to EZO circuit        
      }
      break;
    case ECProbe:
      if (LocalSwitches.ECProbePowered == true) {
        digitalWrite(GPIO_EC_Switch, HIGH);                   // power ON
        digitalWrite(GPIO_EC_isolator, HIGH);                 // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_EC_Switch, LOW);                    // power OFF
        digitalWrite(GPIO_EC_isolator, LOW);                  // ISO-VCC not applied to EZO circuit        
      }
      break;
    case RTDProbe:
      if (LocalSwitches.RTDProbePowered == true) {
        digitalWrite(GPIO_RTD_Switch, HIGH);                  // power ON
        digitalWrite(GPIO_RTD_isolator, HIGH);                // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_RTD_Switch, LOW);                   // power OFF
        digitalWrite(GPIO_RTD_isolator, LOW);                 // ISO-VCC not applied to EZO circuit        
      }
      break;
    case DOProbe:
      if (LocalSwitches.DOProbePowered == true) {
        digitalWrite(GPIO_DO_Switch, HIGH);                   // power ON
        digitalWrite(GPIO_DO_isolator, HIGH);                 // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_DO_Switch, LOW);                    // power OFF
        digitalWrite(GPIO_DO_isolator, LOW);                  // ISO-VCC not applied to EZO circuit        
      }
      break;
    case VEML7700:
      if (LocalSwitches.VEML7700Powered == true) {
        digitalWrite(GPIO_VEML_Switch, HIGH);                 // power ON
        digitalWrite(GPIO_VEML_isolator, HIGH);               // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_VEML_Switch, LOW);                  // power OFF
        digitalWrite(GPIO_VEML_isolator, LOW);                // ISO-VCC not applied to EZO circuit        
      }
      break;
    case DS18B20:
      if (LocalSwitches.DS18B20Powered == true) {
        digitalWrite(GPIO_DS18_Switch, HIGH);                 // power ON
        digitalWrite(GPIO_DS18_isolator, HIGH);               // ISO-VCC applied to EZO circuit
      } else {
        digitalWrite(GPIO_DS18_Switch, LOW);                  // power OFF
        digitalWrite(GPIO_DS18_isolator, LOW);                // ISO-VCC not applied to EZO circuit        
      }
      break;
  }
}



/* ######################################################################################################## */
// END of file
