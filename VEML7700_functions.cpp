
#include      "veml7700_functions.h"

// Local variables used by this module
//boolean                     ProbeLuminosity;
uint16_t                    NbrCounts;                  // luminosity probe VEML7700
uint8_t                     VEML_Gain;
uint8_t                     IntegrationTime;
float                       Measure;
float                       Lux_VEML, Lux_VEML_Norm, White_VEML, White_VEML_Norm;

/* Class instances which are objects with public methods and private attributes */
Adafruit_VEML7700 veml = Adafruit_VEML7700();

/**********************************************************************************************************************************/
/* Function to get only one measure from the VEML7700 probe.                                                                      */
/**********************************************************************************************************************************/
float luxmeter(boolean ProbeReady) {
  bool MaxValuesGain_IT = false;
  uint8_t k;
  NbrCounts = veml.readALS();
  if (ProbeReady == true) {
    do {
      NbrCounts = veml.readALS();
      if (NbrCounts <= 100) {
        //veml.powerSaveEnable(true);
        VEML_Gain = veml.getGain();                         // uint8_t Adafruit_VEML7700::getGain(void) { return ALS_Gain->read(); }
        if (VEML_Gain != VEML7700_GAIN_2) increaseGain(VEML_Gain);
        else {                // here VEML_Gain == VEML7700_GAIN_2
          IntegrationTime = veml.getIntegrationTime();
          if (IntegrationTime != VEML7700_IT_800MS) increaseIntegrationTime(IntegrationTime);
          else {              // here IntegrationTime == VEML7700_IT_800MS
            MaxValuesGain_IT = true;
            break;            // normaly break allows to quit the do while instruction
          }
        }
        //veml.powerSaveEnable(false);
      } else if (NbrCounts > 10000) {     // ALS countings are superior than 10000 => decrease integration time 
        IntegrationTime = veml.getIntegrationTime();
        if (IntegrationTime != VEML7700_IT_25MS) decreaseIntegrationTime(IntegrationTime);
      }
    } while(NbrCounts <= 100 || NbrCounts > 10000);   // the good criterion to get a measure
    NbrCounts = veml.readALS();                       // here gain and integration time have been choosen     
    IntegrationTime = veml.getIntegrationTime();
    VEML_Gain = veml.getGain();
    Measure = veml.readLux();
    //Lux_VEML_Norm = veml.readLuxNormalized();
    //White_VEML = veml.readWhite();
    //White_VEML_Norm = veml.readWhiteNormalized();
    if (MaxValuesGain_IT == true) Serial.println(F("Gain and Integration Time at maximum"));
    Lux_VEML = ConvFunction(NbrCounts, IntegrationTime, VEML_Gain);
    for (k = 0; k < 80; k++) Serial.print('*');
    Serial.print(F("Ambient light sensor in lux (calculated value): "));
    Serial.println(Lux_VEML);
    Serial.print(F("Lux value measured: "));
    Serial.println(Measure);
    for (k = 0; k < 80; k++) Serial.print('*');
    Serial.println();
    //DisplayFeatures();
  }
  return Measure;
}
/**********************************************************************************************************************************/
/* Function to display the most important features of the Ambient Light Sensor.                                                   */
/**********************************************************************************************************************************/
void DisplayFeatures(void) {
  uint8_t k;
  float MyLux;
  Gain_t ProbeGain;
  IntegrationTime_t ProbeIT;
  for (k = 0; k < 80; k++) Serial.print('*');
  NbrCounts = veml.readALS();
  Serial.print(F("\nCounts number: ")); Serial.println(NbrCounts, DEC);
  ProbeGain = (Gain_t)veml.getGain();
  Serial.print(F("Gain: "));
  switch (ProbeGain) {
    case MyVEML7700_GAIN_1:
      Serial.println("ALS gain 1x");
      break;
    case MyVEML7700_GAIN_2:
      Serial.println("ALS gain 2x");
      break;
    case MyVEML7700_GAIN_1_8:
      Serial.println("ALS gain 1/8x");
      break;    
    case MyVEML7700_GAIN_1_4:
      Serial.println("ALS gain 1/4x");
      break;
  }
  ProbeIT = (IntegrationTime_t)veml.getIntegrationTime();
  Serial.print(F("Integration time: "));
  switch (ProbeIT) {
    case MyVEML7700_IT_25MS:
      Serial.println("ALS integration time of 25ms");
      break;
    case MyVEML7700_IT_50MS:
      Serial.println(F("ALS integration time of 50ms"));
      break;
    case MyVEML7700_IT_100MS:
      Serial.println(F("ALS integration time of 100ms"));
      break;
    case MyVEML7700_IT_200MS:
      Serial.println(F("ALS integration time of 200ms"));
      break;
    case MyVEML7700_IT_400MS:
      Serial.println(F("ALS integration time of 400ms"));
      break;    
    case MyVEML7700_IT_800MS:
      Serial.println(F("ALS integration time of 800ms"));
      break;
  }     
  MyLux = ConvFunction(NbrCounts, (uint8_t)ProbeIT, (uint8_t)ProbeGain);
  Serial.print(F("Ambient light sensor in lux (calculated value): ")); Serial.println(MyLux);

  Lux_VEML = veml.readLux();
  Serial.print(F("Ambient light sensor in lux (from device): ")); Serial.println(Lux_VEML);
  Lux_VEML_Norm = veml.readLuxNormalized();
  Serial.print(F("ALS normalized: ")); Serial.println(Lux_VEML_Norm);
  White_VEML = veml.readWhite();
  Serial.print(F("Ambient light sensor white channel: ")); Serial.println(White_VEML);        
  White_VEML_Norm = veml.readWhiteNormalized();
  Serial.print(F("White Channel Normalized: ")); Serial.println(White_VEML_Norm);
  for (k = 0; k < 80; k++) Serial.print('*');
  Serial.println();
}
/**********************************************************************************************************************************/
/* Function to increase the gain of the ambient light sensor VEML7700.                                                            */
/**********************************************************************************************************************************/
void increaseGain(uint8_t GainValue) {
  uint8_t NewGain;
  switch (GainValue) {
    case VEML7700_GAIN_1:
      NewGain = VEML7700_GAIN_2;
      break;
    case VEML7700_GAIN_2:
      break;
    case VEML7700_GAIN_1_8:
      NewGain = VEML7700_GAIN_1_4;
      break;
    case VEML7700_GAIN_1_4:
      NewGain = VEML7700_GAIN_1;
      break;
  }
  veml.setGain(NewGain);
}
/**********************************************************************************************************************************/
/* Function to increase the integration time when counts fron ALS probe is lower than 100.                                        */
/**********************************************************************************************************************************/
void increaseIntegrationTime(uint8_t MyIntegrationTime) {
  uint8_t NewIntegrationTime;
  switch (MyIntegrationTime) {
    case VEML7700_IT_100MS:
      NewIntegrationTime = VEML7700_IT_200MS;
      break;
    case VEML7700_IT_200MS:
      NewIntegrationTime = VEML7700_IT_400MS;
      break;
    case VEML7700_IT_400MS:
      NewIntegrationTime = VEML7700_IT_800MS;
      break;
    case VEML7700_IT_800MS:
      break;
    case VEML7700_IT_50MS:
      NewIntegrationTime = VEML7700_IT_100MS;
      break;
    case VEML7700_IT_25MS:
      NewIntegrationTime = VEML7700_IT_50MS;
      break;
  }
  veml.setIntegrationTime(NewIntegrationTime);  
}
/**********************************************************************************************************************************/
/* Function to decrease the integration time when counts fron ALS probe is upper than 10000.                                      */
/**********************************************************************************************************************************/
void decreaseIntegrationTime(uint8_t MyIntegrationTime) {
  uint8_t NewIntegrationTime;
  switch (MyIntegrationTime) {
    case VEML7700_IT_100MS:
      NewIntegrationTime = VEML7700_IT_50MS;
      break;
    case VEML7700_IT_200MS:
      NewIntegrationTime = VEML7700_IT_100MS;
      break;
    case VEML7700_IT_400MS:
      NewIntegrationTime = VEML7700_IT_200MS;
      break;
    case VEML7700_IT_800MS:
      NewIntegrationTime = VEML7700_IT_400MS;
      break;
    case VEML7700_IT_50MS:
      NewIntegrationTime = VEML7700_IT_25MS;
      break;
    case VEML7700_IT_25MS:
      break;
  }
  veml.setIntegrationTime(NewIntegrationTime);
}
/**********************************************************************************************************************************/
/* Function to calculate the ilumination using the number of counting, the integration time and the gain.                         */
/**********************************************************************************************************************************/
float ConvFunction(uint16_t MyCounts, uint8_t MyIntegrationTime, uint8_t MyVEML_Gain) {
  float lux_veml;
  float resolution;
  switch (MyIntegrationTime) {
    case VEML7700_IT_100MS:
      switch (MyVEML_Gain) {
        case VEML7700_GAIN_1_8:
          resolution = 0.4608;
          break;
        case VEML7700_GAIN_1_4:
          resolution = 0.2304;
          break;  
        case VEML7700_GAIN_1:
          resolution = 0.0576;
          break;
        case VEML7700_GAIN_2:
          resolution = 0.0288;
          break;
      }
      break;
    case VEML7700_IT_200MS:
      switch (MyVEML_Gain) {
        case VEML7700_GAIN_1_8:
          resolution = 0.2304;
          break;
        case VEML7700_GAIN_1_4:
          resolution = 0.1152;
          break;  
        case VEML7700_GAIN_1:
          resolution = 0.0288;
          break;
        case VEML7700_GAIN_2:
          resolution = 0.0144;
          break;
      }
      break;
    case VEML7700_IT_400MS:
      switch (MyVEML_Gain) {
        case VEML7700_GAIN_1_8:
          resolution = 0.1152;
          break;
        case VEML7700_GAIN_1_4:
          resolution = 0.0576;
          break;  
        case VEML7700_GAIN_1:
          resolution = 0.0144;
          break;
        case VEML7700_GAIN_2:
          resolution = 0.0072;
          break;
      }
      break;
    case VEML7700_IT_800MS:
      switch (MyVEML_Gain) {
        case VEML7700_GAIN_1_8:
          resolution = 0.0576;
          break;
        case VEML7700_GAIN_1_4:
          resolution = 0.0288;
          break;  
        case VEML7700_GAIN_1:
          resolution = 0.0072;
          break;
        case VEML7700_GAIN_2:
          resolution = 0.0036;
          break;
      }
      break;
    case VEML7700_IT_50MS:
      switch (MyVEML_Gain) {
        case VEML7700_GAIN_1_8:
          resolution = 0.9216;
          break;
        case VEML7700_GAIN_1_4:
          resolution = 0.4608;
          break;  
        case VEML7700_GAIN_1:
          resolution = 0.1152;
          break;
        case VEML7700_GAIN_2:
          resolution = 0.0576;
          break;
      }
      break;
    case VEML7700_IT_25MS:
      switch (MyVEML_Gain) {
        case VEML7700_GAIN_1_8:
          resolution = 1.8432;
          break;
        case VEML7700_GAIN_1_4:
          resolution = 0.9216;
          break;  
        case VEML7700_GAIN_1:
          resolution = 0.2304;
          break;
        case VEML7700_GAIN_2:
          resolution = 0.1152;
          break;
      }
      break;
  }
  lux_veml = MyCounts * resolution;
  return lux_veml;
}





/* END OF FILE */
