// WhiteBox Labs -- Tentacle Shield -- Circuit Setup -- I2C only!
// https://www.whiteboxes.ch/tentacle
//
// NOTE: This sketch will work only with circuits in I2C mode, e.g. if you're using the Tentacle Mini
// or the Tentacle on an Arduino without SoftSerial (e.g. Zero)
//
// Control commands defined by operator using terminal :
// help, scan, chang_add, mesure, mesure_repeat, stop, temp, cal, calraz
// The aim of this program is to collect measures as the same rule than a multi-parameters probe.
// The probes delivered by Atlas Scientific allow to get a good quality measure to get an assessment of the natural environment.
/* ############################################################################################################## */
// classe String : https://www.arduino.cc/en/Reference/StringObject
/* ############################################################################################################## */
#include      "Functions.h"
#include      "veml7700_functions.h"
#include      "RTC_DS3231.h"

/******************************* local variables *******************************/
float                       Mes_Temp, Mes_pH, Mes_EC, Mes_ORP, Mes_DO, Mes_RTD;
float                       Mes_Lux;

char                        *cmd;                                 // pointer to transform the array of ASCII characters into String format
String                      Command;
uint8_t                     computer_bytes_received;              // We need to know how many characters bytes have been received
char                        computerdata[20];                     // Array to store the commands from terminal
uint8_t                     i, j, k, l, m, n, p, t;
uint8_t                     Ctrl_Flags;
int                         InComingByte;                         // virtual int read(void); method from HardwareSerial.h
boolean                     computer_msg_complete;
uint8_t                     scratch_8bits;
uint16_t                    scratch_16bits;
uint32_t                    scratch_32bits;
SamplingDelay_t             SamplingState;
I2CProbesConnected_t        ConnectedProbes;
ProbeMeasures_t             AllSensorMeasures;
boolean                     DS18B20Present;

/******************************* External variables *******************************/
extern volatile uint16_t    cmpt1, cmpt2, cmpt3, cmpt4, cmpt5, cmpt_5ms;          // Timers
extern volatile uint8_t     compt1, compt2, compt3, compt4, compt5, cmpt_100us;   // Timers

/******************************* Initialization *******************************/
void setup() {
  Serial.begin(115200, SERIAL_8N1);
  while (!Serial);
  cli();
  Ctrl_Flags = ((1<<Timer1_ON)|(1<<Timer2_ON)|(1<<Timer4_ON));
  Init_Timers(Ctrl_Flags, 250, 10000, 200, 2500, 25000, 3125);    // Timer0 .... Timer5, avoid : Timer5_ON and Timer0_ON
  sei();
  initI2C_Devices();                    // Wire.begin()
  Command.reserve(60);
  DS18B20Present = DallasTemperatureSearch(); 
  ConnectedProbes = scani2c();          // it is a mandatory call to find out the available probes with initializing flags
  computer_bytes_received = 0;
  computer_msg_complete = false;
  SamplingState.RepeatedMeasures = false;
}

/******************************* Main loop *******************************/
void loop() {
  while (Serial.available() > 0) {                                  // While there is serial data from the computer
    InComingByte = Serial.read();                                   // read an int type from terminal (virtual int read(void);)
    if (InComingByte == '\n' || InComingByte == '\r') {             // if a newline character arrives, we assume a complete command has been received
      computerdata[computer_bytes_received] = '\0';
      computer_msg_complete = true;
      computer_bytes_received = 0;
    } else computerdata[computer_bytes_received++] = InComingByte;  // command not complete yet, so just ad the byte to data array
  }

  /* Area to interpret the commands from the terminal */
  if (computer_msg_complete == true) {                // if there is a complete command from the computer
    cmd = computerdata;                               // cmd is a pointer and computerdata is an array (cmd = &computerdata[0];)
    Serial.print(F("Command from terminal: "));       // echo to the serial console
    Serial.println(cmd);
    Command = String(cmd);
    if (Command.startsWith("help")) help(Command);                    // if help entered...call help dialogue https://www.arduino.cc/en/Reference/StringConstructor
    if (Command.startsWith("scan")) ConnectedProbes = scani2c();      // if i2c-scan requested
    if (Command.startsWith("chang_add")) change_add_I2C(Command);     // Cde = 'change_add'<105> (between 1 and 127)
    if (Command.startsWith("meas")) AllSensorMeasures = Reading_probes(Command);
    if (Command.startsWith("comp")) CompensatedTemp_pH_DO(Command);   // Temperature compensation when pH probe is calibrate 'comp'<DD.D>
    if (Command.startsWith("cal")) Calibration(Command);              // this command needs to know which probe has got the focus
    if (Command.startsWith("delete")) DeleteCalibration(Command);
    if (Command.startsWith("calpoint")) DisplayNbrCalPoints(Command);
    if (Command.startsWith(F("lux"))) Mes_Lux = luxmeter(ConnectedProbes.VEML7700_Probe);
    if (Command.startsWith(F("orp"))) Mes_ORP = orpMeasure(ConnectedProbes.ORP_Probe);
    if (Command.startsWith(F("cond"))) Mes_EC = ConductivityMeasure(ConnectedProbes.EC_Probe);
    if (Command.startsWith(F("ph"))) Mes_pH = pHMeasure(ConnectedProbes.pH_Probe);
    if (Command.startsWith(F("oxy"))) Mes_DO = OxyMeasure(ConnectedProbes.DO_Probe);
    if (Command.startsWith(F("temp"))) Mes_Temp = TempMeasure(DS18B20Present);
    if (Command.startsWith(F("token"))) CheckSetFocus(Command);       // 'token?' or 'token_'<probe>, probe is ph, oxy, orp, cond, rtd or none  
    if (Command.startsWith(F("repeat"))) SamplingState = SamplingDelayMeasure(Command, SamplingState);

    if (Command.startsWith("cfgtime_")) Change_heure(Command);        // 'cfgtime_'<hhmmss>
    if (Command.startsWith("cfgdate_")) Change_date(Command);         // 'cfgdate_'<aaaammjj>
    if (Command.startsWith("cfgdow_")) Change_DoW(Command);           // 'cfgdow_'<d>
    if (Command.startsWith("lecture")) lecture();
    if (Command.startsWith("helprtc")) helprtc();                     // spécifique à l'horloge RTC
    computer_msg_complete = false;    // Reset the var computer_bytes_received to equal 0
  }


  /* Area to apply actions on the probes, on the communication flows or to define a sampling interval */
  if (SamplingState.RepeatedMeasures == true) {
    
  }

}





/* ######################################################################################################## */
// END of file
