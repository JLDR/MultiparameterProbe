/* ******************************************************************************************** */
/* Functions availables for Ambient Light Sensor VEML7700.                                      */
/* ******************************************************************************************** */
#ifndef VEML7700_FUNCTIONS_H_
#define VEML7700_FUNCTIONS_H_         1

#include      <Adafruit_VEML7700.h>
#include      <string.h>
#include      <stdio.h>
#include      <stdint.h>
#include      <stdlib.h>



/********************************************** Predefined types **********************************************/
typedef enum AvailableGains : uint8_t {
  MyVEML7700_GAIN_1 = 0x00,           ///< ALS gain 1x
  MyVEML7700_GAIN_2 = 0x01,           ///< ALS gain 2x
  MyVEML7700_GAIN_1_8 = 0x02,         ///< ALS gain 1/8x
  MyVEML7700_GAIN_1_4 = 0x03          ///< ALS gain 1/4x
} Gain_t;

typedef enum AvailableIT : uint8_t {
  MyVEML7700_IT_100MS = 0x00,         ///< ALS intetgration time 100ms
  MyVEML7700_IT_200MS = 0x01,         ///< ALS intetgration time 200ms
  MyVEML7700_IT_400MS = 0x02,         ///< ALS intetgration time 400ms
  MyVEML7700_IT_800MS = 0x03,         ///< ALS intetgration time 800ms
  MyVEML7700_IT_50MS = 0x08,          ///< ALS intetgration time 50ms
  MyVEML7700_IT_25MS = 0x0C           ///< ALS intetgration time 25ms
} IntegrationTime_t;

/* Function prototype or functions interface */
float luxmeter(boolean);
void DisplayFeatures(void);
void increaseGain(uint8_t);
void increaseIntegrationTime(uint8_t);
void decreaseIntegrationTime(uint8_t);
float ConvFunction(uint16_t, uint8_t, uint8_t);




#endif /* VEML7700_FUNCTIONS_H_ */



/* END FILE */
