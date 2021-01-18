
#include    "RTC_DS3231.h"

// Variables externes

// Variables locales propres au module
char                        ConvAsciiToInt[5];            // tableau de '0' à "65535"
char                        CdeAsciiArray[40];
uint8_t                     heures, minutes, secondes, DOW, jour, mois, short_annee;
uint16_t                    annee;
uint8_t                     HoursInRTC, MinutesInRTC, SecondsInRTC, DayInRTC, MonthInRTC, YearInRTC;
uint8_t                     check_date, check_month, check_year;
uint8_t                     check_heure, check_minutes;
uint8_t                     cycle_saison;   // contient 0x5B ou 0xA4
uint8_t                     jour_ref;       // jour pour lequel il y aura changement d'horaire soit à 3H00 (horaire d'hiver) soit à 2H00 (horaire d'été)
static const uint8_t        Months[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

uint8_t                     drapeaux_maj;
uint32_t                    temps_UNIX;     // from -2,147,483,648 to 2,147,483,647 (-2^31 to 2^31 - 1)
char                        DateArray[10];
EndianFormat_t              Endianness;     // endianisme ou boutisme
FormatType_t                ShortLong;      // SHORT or LONG format
bool                        Century = false;
uint32_t                    scratchRTC_32bits;
uint16_t                    scratchRTC_16bits;
uint8_t                     scratchRTC_8bits;
uint16_t                    lecture_masqueRTC;

/* Class instances which are objects with public methods and private attributes */
static Eeprom24C32_64 eeprom(EEPROM_ADDRESS);   // https://forum.arduino.cc/index.php?topic=38143.0
RTClib MyRTC;
DateTime MyTime;        // DateTime is a class and MyTime is an object of this class
DS3231 MylocalRTC;

/****************************************************************************************************/
/* We retrieve the parameters which are contained in the control command: 'cfgtime_'<hhmmss>.       */
/****************************************************************************************************/
void Change_heure(String Cde_received) {                                  // cfgtime_hhmmss
  uint8_t k = 0;
  MylocalRTC.setClockMode(false);                                         // 24 hours cycle
  memset(CdeAsciiArray, '\0', sizeof(CdeAsciiArray));
  memset(ConvAsciiToInt, '\0', sizeof(ConvAsciiToInt));
  Cde_received = Cde_received.substring(8);                               // to remove the text "cfgtime_"
  Cde_received.toCharArray(CdeAsciiArray, Cde_received.length() + 1);     // useful to the end of the array '\0'
  ConvAsciiToInt[0] = CdeAsciiArray[k++];
  ConvAsciiToInt[1] = CdeAsciiArray[k++];
  ConvAsciiToInt[2] = '\0';
  heures = Conv2AsciiCharToUint8_t(&ConvAsciiToInt[0]);
  ConvAsciiToInt[0] = CdeAsciiArray[k++];
  ConvAsciiToInt[1] = CdeAsciiArray[k++];
  ConvAsciiToInt[2] = '\0';
  minutes = Conv2AsciiCharToUint8_t(&ConvAsciiToInt[0]);  
  ConvAsciiToInt[0] = CdeAsciiArray[k++];
  ConvAsciiToInt[1] = CdeAsciiArray[k++];
  ConvAsciiToInt[2] = '\0';
  secondes = Conv2AsciiCharToUint8_t(&ConvAsciiToInt[0]);
  if (heures <= 23 && heures >= 0) {
    if (minutes <= 59 && minutes >= 0) {
      if (secondes <= 59 && secondes >= 0) {
        MylocalRTC.setSecond((byte)secondes);
        MylocalRTC.setMinute((byte)minutes);
        MylocalRTC.setHour((byte)heures);
        drapeaux_maj |= (1<<flag_maj_heure);
      } else Serial.println(F("The second value must be a digit from 0 to 59 included"));
    } else Serial.println(F("The minute value must be a digit from 0 to 59 included"));
  } else Serial.println(F("The hour value must be a digit from 0 to 23 included"));
}
/****************************************************************************************************/
/* The function below is never called between 2H00 and 3H00 of the morning, those involves that a   */
/* flag is activated to define the winter or summer period. Writing from main program the 0x5B value*/
/* at addresses 0x200 or 0x201.                                                                     */
/* Control command from the terminal is: 'cfgdate_'<aaaammjj>                                       */
/****************************************************************************************************/
void Change_date(String Cde_received) {
  uint8_t k = 0;
  memset(CdeAsciiArray, '\0', sizeof(CdeAsciiArray));
  memset(ConvAsciiToInt, '\0', sizeof(ConvAsciiToInt));
  Cde_received = Cde_received.substring(8);
  Cde_received.toCharArray(CdeAsciiArray, Cde_received.length() + 1);
  ConvAsciiToInt[0] = CdeAsciiArray[k++];
  ConvAsciiToInt[1] = CdeAsciiArray[k++];
  ConvAsciiToInt[2] = CdeAsciiArray[k++];                     // those digits which interest us
  ConvAsciiToInt[3] = CdeAsciiArray[k++];
  ConvAsciiToInt[4] = '\0';
  annee = Conv5AsciiCharToUint16_t(&ConvAsciiToInt[0]);       // long format
  short_annee = Conv2AsciiCharToUint8_t(&ConvAsciiToInt[2]);  // short format
  check_year = Conv2AsciiCharToUint8_t(&CdeAsciiArray[2]);    // used to calculate the new time when the it changes 
  ConvAsciiToInt[0] = CdeAsciiArray[k++];
  ConvAsciiToInt[1] = CdeAsciiArray[k++];
  ConvAsciiToInt[2] = '\0';
  mois = Conv2AsciiCharToUint8_t(&ConvAsciiToInt[0]);
  check_month = Conv2AsciiCharToUint8_t(&CdeAsciiArray[4]);
  ConvAsciiToInt[0] = CdeAsciiArray[k++];
  ConvAsciiToInt[1] = CdeAsciiArray[k++];
  ConvAsciiToInt[2] = '\0';
  jour = Conv2AsciiCharToUint8_t(&ConvAsciiToInt[0]);
  check_date = Conv2AsciiCharToUint8_t(&CdeAsciiArray[6]);
  if (annee > 2000 && annee <= 2040) {
    if (mois > 0 && mois <= 12) {
      if (jour > 0 && jour <= 31) {
        MylocalRTC.setDate((byte)jour);
        MylocalRTC.setMonth((byte)mois);
        MylocalRTC.setYear((byte)short_annee);
      }
    }
  }
  if (check_year >= 18 && check_year <= 99) {
    drapeaux_maj |= (1<<flag_maj_date);         // lorsque la carte démarre ce drapeau est supprimé
    Serial.print(F("To check the year value stored: "));
    Serial.println(check_year, DEC);
    Serial.print(F("To check the month value stored: "));
    Serial.println(check_month, DEC);
    Serial.print(F("To check the date value stored: "));
    Serial.println(check_date, DEC);
  }
}
/****************************************************************************************************/
/* This function set the date of the week choosing a number between 1 and 7.                        */
/* The control command is : 'cfgdow_'<d>                                                            */
/* Monday is the first date of the week and Sunday is the 7th and last day of the week.             */
/****************************************************************************************************/
void Change_DoW(String Cde_received) {
  byte DayOfTheWeek;
  memset(CdeAsciiArray, '\0', sizeof(CdeAsciiArray));
  Cde_received = Cde_received.substring(7);
  Cde_received.toCharArray(CdeAsciiArray, Cde_received.length() + 1);
  DayOfTheWeek = (byte)((uint8_t)CdeAsciiArray[0] - 0x30);
  if (DayOfTheWeek > 0 && DayOfTheWeek <= 7) MylocalRTC.setDoW(DayOfTheWeek);
}
/********************************************************************************************************/
/* Reading of the register values of the RTC circuit DS3231.                                            */
/* difference between FORMAT_SHORT and FORMAT_LONG => 17 becomes 2017                                   */
/* FORMAT_LITTLEENDIAN => 13/04/2017, FORMAT_BIGENDIAN => 2017/04/13, FORMAT_MIDDLEENDIAN => 04/13/2017 */
/* The function Affiche_heure updates HoursInRTC, MinutesInRTC et SecondsInRTC.                         */
/* Reading time, we retrieve a pointer which has been initialized on the first position of a string.    */
/********************************************************************************************************/
void lecture() {
  uint8_t var, k;
  char *ptr;
  Affiche_heure();
  separateur1(61, '-');
  Serial.println(F("lecture of the content of the RTC circuit : day, month and year"));
  separateur1(61, '-');
  DayInRTC = (uint8_t)MylocalRTC.getDate();
  Serial.print(F("day: ")); Serial.println(DayInRTC, DEC);
  MonthInRTC = (uint8_t)MylocalRTC.getMonth(Century);
  Serial.print(F("month: ")); Serial.println(MonthInRTC, DEC);
  YearInRTC = (uint8_t)MylocalRTC.getYear();
  Serial.print(F("year: ")); Serial.print(YearInRTC, DEC);
  DOW = (uint8_t)MylocalRTC.getDoW();
  Serial.print(F("\n- Date Of the Week: "));
  Serial.print(DOW, DEC);
//  DayOfTheWeek = MyTime.dayOfTheWeek();
//  Serial.print(F("\n- Date Of the Week from DateTime object: "));
//  Serial.print(DayOfTheWeek, DEC);
  Serial.print(F("\n- Unix Time: "));
  temps_UNIX = MyTime.unixtime();
  Serial.println(temps_UNIX);
}
/****************************************************************************************************/
/* To display the time. MyRTC is an object of the RTClib class. MyTime is an object of the DateTime */
/* class. This function updates commun variables HoursInRTC, MinutesInRTC et SecondsInRTC.          */
/****************************************************************************************************/
void Affiche_heure() {
  MyTime = MyRTC.now();
  Serial.print(F("Heure lue : "));
  HoursInRTC = MyTime.hour();                    // uint8_t (hh)
  if (HoursInRTC <= 9) {
    Serial.print('0');
    Serial.print((char)(HoursInRTC + 0x30));
  } else Serial.print(HoursInRTC, DEC);
  Serial.print(':');
  MinutesInRTC = MyTime.minute();                 // uint8_t (mm)
  if (MinutesInRTC <= 9) {
    Serial.print('0');
    Serial.print((char)(MinutesInRTC + 0x30));
  } else Serial.print(MinutesInRTC, DEC);
  Serial.print(':');
  SecondsInRTC = MyTime.second();                // ss
  if (SecondsInRTC <= 9) {
    Serial.print('0');
    Serial.println((char)(SecondsInRTC + 0x30));
  } else Serial.println(SecondsInRTC, DEC);
}
/****************************************************************************************************/
/* We convert an array of 2 ASCII characters into an unsigned integer of 8 bits to retrieve a value */
/* from 0 to 99. the first parameter transmitted is the most significative digit.                   */
/****************************************************************************************************/
uint8_t Conv2AsciiCharToUint8_t(char *ptr_ascii) {
  uint8_t scratch_8bits = 0;
  uint8_t n;
  for (n = 0; n < 2; n++) {     // pour 2 caractères ASCII représentant un nombre de 0 à 99
    if ((uint8_t)(*ptr_ascii) < 0x30 || (uint8_t)(*ptr_ascii) > 0x39) break;    // rupture de la boucle for
    else {
      if (n == 0) {
        scratch_8bits = ((uint8_t)(*ptr_ascii) - 0x30) * 10;        // Most significant byte
        ptr_ascii++;
      } else scratch_8bits += (uint8_t)(*ptr_ascii) - 0x30;
    }
  }
  if (scratch_8bits <= 1 && scratch_8bits >= 100) return 0;          // 2002 - 2099
  else return scratch_8bits;
}
/****************************************************************************************************/
/* Text divider                                                                                     */
/****************************************************************************************************/
void separateur1(uint8_t nbr_carac, char caract) {
  uint8_t i;
  for (i = 0; i < nbr_carac; i++) {
    Serial.print(caract);
  }
  Serial.println();
}
/****************************************************************************************************/
/* Conversion of an array which represent an ASCII string with a maximum of 5 characters.           */
/* The integer sended is a 16 bits unsigned (<65536).                                               */
/****************************************************************************************************/
uint16_t Conv5AsciiCharToUint16_t(char *ptr_array) {
  uint8_t n = 0;      // doit permettre d'identifier les caractères ASCII interprétables
  uint32_t entier_verif = 0UL;
  uint16_t result_local;
  char *ptr_local;
  uint8_t i;
  ptr_local = ptr_array;
  do {
    if ((uint8_t)(*ptr_array) < 0x30 || (uint8_t)(*ptr_array) > 0x39) break;    // rupture de la boucle do while
    else {
      Serial.print(*(ptr_array++));
      n++;            // nombre de caractères interprétables
    }
  } while(1);
  if (n != 0) {
    ptr_local += (n - 1) * sizeof(char);      // on fixe la position de l'unité dans le tableau
    for (i = 0; i < n; i++) {
      switch(i) {
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
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 1000
          break;
        case 4:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 10000
          break;
        case 5:
          entier_verif += (uint32_t)((*(ptr_local--) - 0x30) * pow(10, i));   // 100000
          break;
        default:
          break;
      }
    }
  }
  if (entier_verif >= 65536) return 0;
  else return (uint16_t)entier_verif;
}
/****************************************************************************************************/
/* Menu Help to identify all control commands allowed with the DS3231 RTC circuit.                  */
/****************************************************************************************************/
void helprtc() {                         //print help dialogue
  separateur1(92, '=');
  Serial.println(F("- Les caractères < et > ne doivent pas être envoyés, ils précisent le paramètre contenu à l'intérieur"));
  Serial.println(F("- La commande est précisée entre apostrophe (qui ne doit pas être utilisée) avec symbole : '"));
  Serial.println(F("- Pour lire les adresses I2C courantes des modules connectés : 'scan'"));
  Serial.println(F("- Pour modifier l'heure avec minutes et secondes : 'cfgtime_'<hhmmss>"));
  Serial.println(F("- Pour modifier la date (année, mois et jour) : 'cfgdate_'<aaaammjj>"));
  Serial.println(F("- Pour modifier le paramètre DoW (Date of the Week) compris entre 1 (lundi) et 7 dimanche) : 'cfgdow_'<d>"));
  Serial.println(F("- Pour lire l'ensemble des paramètres contenus dans l'horloge RTC : 'lecture'"));
  Serial.println(F("- Pour définir le format de lecture, on utilise la commande : 'format_'<xx>"));
  Serial.println(F("\t\t\t - avec deux chiffres :"));
  Serial.println(F("\t\t\t\t - le premier :\n\t\t\t\t\t\tFORMAT SHORT : 1\n\t\t\t\t\t\tFORMAT LONG : 2"));
  Serial.println(F("\t\t\t\t - le second :\n\t\t\t\t\t\tFORMAT LITTLEENDIAN : 1\n\t\t\t\t\t\tFORMAT BIGENDIAN : 2\n\t\t\t\t\t\tFORMAT MIDDLEENDIAN : 3"));
  Serial.println(F("- Pour lire le format : 'format?'"));
  Serial.println(F("- Pour modifier le jour en cours : 'jour' ou 'jour_'<x> avec x de 1 à 7"));
  Serial.println(F("\nCommande pour tester la fonction de conversion :"));
  Serial.println(F("- Pour lire un entier 16 bits à partir d'un tableau de type char : 'conv_'<x> avec x de 0 à 6"));
  Serial.println(F("- Pour modifier la variable (uint8_t) Indice : 'indice_'<x> avec x de 0 à 5"));
  Serial.println(F("- Pour écrire un octet en eeprom à l'adresse comprise entre 0 et 0xFFF : 'WRbyte_'<XXXxx>"));
  Serial.println(F("- Pour lire un octet en eeprom à l'adresse comprise entre 0 et 0xFFF : 'RDbyte_'<XXX>"));
  Serial.println(F("- Pour vérifier la conversion hexadécimale d'une chaîne HexASCII: 'Conv_'<XXXX>"));
  separateur1(92, '=');
//  cout_Fonctions << F("CONFIGURATION de l'horloge Real Time Clock (DS3231) :\n");
//  cout_Fonctions << F("- Pour lire le contenu de l'horloge RTC : 'lect'\n");
//  cout_Fonctions << F("- Pour modifier l'heure : 'cfgtime_'<hhmmss>\n");
//  cout_Fonctions << F("- Pour modifier la date : 'cfgdate_'<aaaammjj>\n");
//  cout_Fonctions << F("- Pour imposer un jour de la semaine : 'jour_'<d>\n");
//  cout_Fonctions << F("\t\tLundi -> 1,\tmardi -> 2,\tmercredi -> 3,\tjeudi -> 4,\n\t\tvendredi -> 5,\tsamedi -> 6,\tdimanche -> 7 \n");
//  cout_Fonctions << F("- Pour consulter le jour de la semaine : 'jour_'<?>\n");
//  cout_Fonctions << F("- Pour modifier automatiquement le jour de la semaine (DOW) en fonction de la date : 'jour'\n");
}
/****************************************************************************************************/
/* Fonction pour afficher tous les paramètres liés au temps restant jusqu'au 31 décembre.           */
/****************************************************************************************************/
void DisplayTerminalRemainingTime() {
  uint8_t k;
  uint8_t RemainingDaysOfTheMonth;
  uint8_t RemainingDaysOfTheYear = 0;
  uint8_t RemainingHours;
  uint8_t RemainingMinutes;
  uint8_t RemainingSeconds;
  bool h12 = true;               // pour distinguer AM/PM
  bool PM = true;
  mois = (uint8_t)MylocalRTC.getMonth(Century);
  jour = (uint8_t)MylocalRTC.getDate();
  RemainingDaysOfTheMonth = Months[mois - 1] - jour;
  for (k = mois; k < 12; k++) RemainingDaysOfTheYear += Months[k];
  RemainingDaysOfTheYear += RemainingDaysOfTheMonth;
  Serial.print(F("Number of days of the month: ")); Serial.println(Months[mois - 1], DEC);
  Serial.print(F("Remaining days of the year: ")); Serial.println(RemainingDaysOfTheYear, DEC);
  Serial.print(F("Remaining days of the month: ")); Serial.println(RemainingDaysOfTheMonth, DEC);
  heures = (uint8_t)MylocalRTC.getHour(h12, PM);                    // byte DS3231::getHour(bool &h12, bool &PM) {...}
  RemainingHours = 23 - heures;
  Serial.print(F("Remaining hours: ")); Serial.println(RemainingHours, DEC);
  minutes = (uint8_t)MylocalRTC.getMinute();
  RemainingMinutes = 59 - minutes;
  Serial.print(F("Remaining minutes: ")); Serial.println(RemainingMinutes, DEC);
  secondes = (uint8_t)MylocalRTC.getSecond();
  RemainingSeconds = 59 - secondes;
  Serial.print(F("Secondes restantes : ")); Serial.println(RemainingSeconds, DEC);
}
/****************************************************************************************************/
/* Fonction pour identifier le nombre de jours restants jusqu'au 31 décembre inclus.                */
/****************************************************************************************************/
uint8_t RemainingNbrDays() {
  uint8_t k;
  uint8_t RemainingDaysOfTheMonth;
  uint8_t RemainingDaysOfTheYear = 0;
  mois = (uint8_t)MylocalRTC.getMonth(Century);           // from 1 to 12
  jour = (uint8_t)MylocalRTC.getDate();                   // from 1 to 7
  RemainingDaysOfTheMonth = Months[mois - 1] - jour;
  if (mois != 12) {
    for (k = mois; k < 12; k++) RemainingDaysOfTheYear += Months[k];
  }
  RemainingDaysOfTheYear += RemainingDaysOfTheMonth;
  return RemainingDaysOfTheYear;
}
/****************************************************************************************************/
/* Function to identify the remainng time of the current day.                                       */
/****************************************************************************************************/
void RemainingTime(char *ptrArrayMain) {
  uint8_t k;
  char DecToAscii[2];
  uint8_t RemainingHours;
  uint8_t RemainingMinutes;
  uint8_t RemainingSeconds;
  bool h12 = true;               // pour distinguer AM/PM
  bool PM = true;
  heures = (uint8_t)MylocalRTC.getHour(h12, PM);      // byte DS3231::getHour(bool &h12, bool &PM) {...}
  RemainingHours = 23 - heures;
  if (RemainingHours <= 9) {
    sprintf(&DecToAscii[0], "%1d", RemainingHours);
    *(ptrArrayMain++) = 0x30;
    *(ptrArrayMain++) = DecToAscii[0];
  } else {
    sprintf(&DecToAscii[0], "%2d", RemainingHours);
    for (k = 0; k < 2; k++) *(ptrArrayMain++) = DecToAscii[k];
  }
  //*(ptrArrayMain++) = 0x3A;
  *(ptrArrayMain++) = 0x2E;
  minutes = (uint8_t)MylocalRTC.getMinute();
  RemainingMinutes = 59 - minutes;
  if (RemainingMinutes <= 9) {
    sprintf(&DecToAscii[0], "%1d", RemainingMinutes);
    *(ptrArrayMain++) = 0x30;
    *(ptrArrayMain++) = DecToAscii[0];
  } else {
    sprintf(&DecToAscii[0], "%2d", RemainingMinutes);
    for (k = 0; k < 2; k++) *(ptrArrayMain++) = DecToAscii[k];
  }
  //*(ptrArrayMain++) = 0x3A;
  *(ptrArrayMain++) = 0x2E;
  secondes = (uint8_t)MylocalRTC.getSecond();
  RemainingSeconds = 59 - secondes;
  if (RemainingSeconds <= 9) {
    sprintf(&DecToAscii[0], "%1d", RemainingSeconds);
    *(ptrArrayMain++) = 0x30;
    *(ptrArrayMain++) = DecToAscii[0];
  } else {
    sprintf(&DecToAscii[0], "%2d", RemainingSeconds);
    for (k = 0; k < 2; k++) *(ptrArrayMain++) = DecToAscii[k];
  }
  *ptrArrayMain = Null;
}













/* ######################################################################################################## */
// END of file







/*          */
