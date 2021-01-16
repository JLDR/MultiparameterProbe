/* ******************************************************************************************** */
/* Methods from the official library supplied by Arduino.                                       */
/* https://www.arduinolibraries.info/libraries/ds3231                                           */
/* ******************************************************************************************** */
#ifndef RTC_DS3231_H_
#define RTC_DS3231_H_     1

/* contrairement au PSoC, c'est bien à cet endroit qu'il faut déclarer les appels des autres bibliothèques */
#include      <Wire.h>
#include      <DS3231.h>
#include      <Eeprom24C32_64.h>


/* constantes */
#define       Space                 0x20
#define       Null                  '\0'              // 0 ou '\0'
#define       F_cpu                 16000000UL
#define       LF                    0x0A              // '\n'
#define       CR                    0x0D              // '\r'
#define       SQW_RATE_1            0                 // void enableOscillator(bool TF, bool battery, byte frequency); 
#define       SQW_RATE_1K           1                 // oscillator OFF/ON, battery OFF/ON, frequency 0 => 1 Hz
#define       SQW_RATE_4K           2                 // 1 => 1.024 KHz, 2 => 2.048 KHz, 3 => 8,192 KHz (default)
#define       SQW_RATE_8K           3
#define       EEPROM_ADDRESS        0x57
#define       RTC_ADDRESS           0x68
#define       code_maj              0x5B
#define       Winter_address        0x200
#define       Summer_address        0x201
#define       flag_hr_march_mod     0       // drapeau activé au mois de mars (drapeaux_maj)
#define       flag_hr_octob_mod     1       // drapeau activé au mois d'octobre
#define       flag_maj_date         2       // permet de confirmer que l'opérateur à mis à jour le jour, le mois et l'année
#define       flag_maj_heure        3       // permet de confirmer que l'opérateur à mis à jour l'heure, les minutes et les secondes
#define       flag_Winter           4       // période d'octobre à mars
#define       flag_Summer           5       // période de mars à octobre
#define       flag_10mns            6
#define       flag_40mns            7


/* Types prédéfinis et énuméartions */
typedef enum EndianFormat {
  Format_LittleEndian = 1,
  Format_BigEndian,
  Format_MiddleEndian
} EndianFormat_t;

typedef enum FormatType {
  FormatShort = 1,
  FormatLong
} FormatType_t;

typedef enum SqwRate {
  Sqw_Rate_1 = 0,
  Sqw_Rate_1k,
  Sqw_Rate_4k,
  Sqw_Rate_8k
} SqwRate_t;

typedef struct Representation {
  EndianFormat_t MyEndianness;
  FormatType_t MyShortLong;
} Representation_t;
     
/* MACRO */
// helper macro : fixe la commande et la sortie produite
#define       LINE(nom,val) Serial.print(nom); Serial.print("\t\t\t"); Serial.println(val);     // https://gist.github.com/ah01/762576

/* Prototypage des fonctions */
void Change_heure(String);
void Change_date(String);
void Change_DoW(String);
void lecture(void);
void Affiche_heure(void);
uint8_t Conv2AsciiCharToUint8_t(char *);
void separateur1(uint8_t, char);
uint16_t Conv5AsciiCharToUint16_t(char *);
void helprtc(void);
void DisplayTerminalRemainingTime(void);
uint8_t RemainingNbrDays(void);
void RemainingTime(char *);



#endif /* FONCTIONS_H_ */
