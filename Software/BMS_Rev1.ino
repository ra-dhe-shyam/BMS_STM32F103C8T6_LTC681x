/*Configuration Mode
  1.  Initialize parameters
  2.  Start the loop
  3.  Measure Voltage, Current, Temperature
  4.  Check faults
  5.  Display parameters on serial monitor
  6.  Check for user input for configuration
  7.  If user input, display the configurable parameters and readings and enable configuration from user input
  8.  If configuration enabled, change the parameters according to user values.
  9.
*/
#include <Arduino.h>
#include <EEPROM.h>
//#include <stdint.h>
#include <SPI.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6812.h"

/************************* Defines *****************************/
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0
#define PWM 1
#define SCTL 2
#define MOSI2 PB15
#define MISO2 PB14
#define SCLK2 PB13
#define SS2 PB12
#define FAULT_VOLTAGE 1
#define FAULT_CURRENT 2
#define FAULT_TEMPERATURE 3
#define FAULT_MEASUREMENT 4
#define DISCHARGE_ENABLE 13
#define CHARGE_ENABLE 12

/********************Function Definition************************/
void Config();
void MeasureVoltage();
void MeasureCurrent();
void MeasureTemp();
void OverVoltage();
void UnderVoltage();
void OverTemp();
void OverCurrent();
void CSOpenWire();
void CellOpenWire();
void TempOpenWire();
void CSOpenWire();

void Fault();
void DisplayAll();
void DisplayMeasured();
void DisplayVoltage();
void DisplayCurrent();
void DisplayTemp();
void check_error(int error);

/********************Class Declaration**************************/
//SPIClass(uint8_t mosi, uint8_t miso, uint8_t sclk, uint8_t ssel)
SPIClass SPI_2(MOSI2, MISO2, SCLK2, SS2);
/***************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
********************************************************************/
const uint8_t TOTAL_IC = 1; //!< Number of ICs in the daisy chain

uint8_t ADC_OPT = ADC_OPT_DISABLED; //!< ADC Mode option bit
uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //!< ADC Mode
uint8_t ADC_DCP = DCP_ENABLED; //!< Discharge Permitted
uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; //!< Channel Selection for ADC conversion
uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;  //!< Channel Selection for ADC conversion
uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;  //!< Channel Selection for ADC conversion
uint8_t SEL_ALL_REG = REG_ALL; //!< Register Selection
uint8_t SEL_REG_A = REG_1; //!< Register Selection
uint8_t SEL_REG_B = REG_2; //!< Register Selection
int8_t error;
uint16_t address = 0 ;
/********************Global BMS Variable***********************/
cell_asic BMS_IC[TOTAL_IC];

//Under Voltage and Over Voltage Thresholds
uint16_t UV_THRESHOLD = EEPROM.read(0)*1000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)
uint16_t OV_THRESHOLD = EEPROM.read(1)*1000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
float OC_THRESHOLD = EEPROM.read(2);
float OT_THRESHOLD = EEPROM.read(3);
uint8_t total_ic = 1;
float Voltage_Input;
bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool GPIOBITS_A[5] = {false, false, true, true, true}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
bool GPIOBITS_B[4] = {false, false, false, false}; //!< GPIO Pin Control // Gpio 6,7,8,9
uint16_t UV = UV_THRESHOLD; //!< Under voltage Comparison Voltage
uint16_t OV = OV_THRESHOLD; //!< Over voltage Comparison Voltage
bool DCCBITS_A[12] = {false, false, false, false, false, false, false, false, false, false, false, false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool DCCBITS_B[7] = {false}; //!< Discharge cell switch //Dcc 0,13,14,15
bool DCTOBITS[4] = {false, true, false, false}; //!< Discharge time value //Dcto 0,1,2,3  // Programed for 4 min
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */
bool FDRF = false; //!< Force Digital Redundancy Failure Bit
bool DTMEN = true; //!< Enable Discharge Timer Monitor
bool PSBits[2] = {false, false}; //!< Digital Redundancy Path Selection//ps-0,1
int MODE_CONTROL = 13;
float Current;
char input;
bool configflag = 0;

void setup()
{
  SPI_2.begin();
  Serial.begin(115200);
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV16);
  LTC6812_init_cfg(TOTAL_IC, BMS_IC);
  LTC6812_init_cfgb(TOTAL_IC, BMS_IC);
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    LTC6812_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS, UV, OV);
    LTC6812_set_cfgrb(current_ic, BMS_IC, FDRF, DTMEN, PSBits, GPIOBITS_B, DCCBITS_B);
  }
  LTC6812_reset_crc_count(TOTAL_IC, BMS_IC);
  LTC6812_init_reg_limits(TOTAL_IC, BMS_IC);
  pinMode(53, OUTPUT);
  pinMode(DISCHARGE_ENABLE, OUTPUT);
  pinMode(CHARGE_ENABLE, OUTPUT);

}

void loop() {
  Config();
  digitalWrite(MODE_CONTROL, LOW);

  MeasureVoltage(); 
  MeasureCurrent();
  MeasureTemp();
  if (Serial.available()) {
    delay(50);
    char input = read_char();
    if (input == 'c') {
      configflag = 1;
      DisplayAll();
    }
    if (input == 'e')  configflag = 0;


  }
  if (!configflag ) {
    DisplayMeasured();
    configflag = 0;
  }
  else {
    DisplayAll();
  }
}
void Config() {
  for (uint8_t cic = 0; cic < TOTAL_IC; cic++)
  {
    LTC6812_set_cfgr(cic, BMS_IC, REFON, ADCOPT, GPIOBITS_A, DCCBITS_A, DCTOBITS, UV_THRESHOLD, OV_THRESHOLD);
    LTC6812_set_cfgrb(cic, BMS_IC, FDRF, DTMEN, PSBits, GPIOBITS_B, DCCBITS_B);
  }
  wakeup_sleep(TOTAL_IC);
  LTC6812_wrcfg(TOTAL_IC, BMS_IC); // Write into Configuration Register
  LTC6812_wrcfgb(TOTAL_IC, BMS_IC); // Write into Configuration Register B

  wakeup_idle(TOTAL_IC);
  error = LTC6812_rdcfg(TOTAL_IC, BMS_IC); // Read Configuration Register
  check_error(error);
  error = LTC6812_rdcfgb(TOTAL_IC, BMS_IC); // Read Configuration Register B
  check_error(error);
}

void MeasureVoltage() {

  wakeup_sleep(TOTAL_IC);
  LTC6812_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
  LTC6812_pollAdc();
  delay(10);
  wakeup_idle(TOTAL_IC);
  LTC6812_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
  check_error(error);
  delay(10);
  wakeup_idle(TOTAL_IC);
  LTC6812_adstat(ADC_CONVERSION_MODE, STAT_CH_TO_CONVERT);
  LTC6812_pollAdc();
  delay(10);
  wakeup_idle(TOTAL_IC);;
  error = LTC6812_rdstat(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
  check_error(error);
  delay(10);
  OverVoltage();
  UnderVoltage();
  CellOpenWire();

}
void MeasureTemp() {

  wakeup_sleep(TOTAL_IC);
  LTC6812_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
  LTC6812_pollAdc();
  delay(10);
  wakeup_idle(TOTAL_IC);
  error = LTC6812_rdaux(SEL_ALL_REG, TOTAL_IC, BMS_IC);
  check_error(error);
  for (int cic = 0; cic < total_ic; cic++) {
    for (int i = 0; i < 4; i++) {
      float resistance = ((50000.0 - (BMS_IC[cic].aux.a_codes[i] * 10000.0)) / 5.0);
      float Temperature = 1.0 / (0.003354 + 0.000257 * log(resistance / 10000.0) + 0.0000026 * log(pow((resistance / 10000.0), 2)) + 0.0000063 * log(pow((resistance / 10000.0) , 3))) - 273.15;
      if (Temperature > OT_THRESHOLD || Temperature < 15.0) {
        Fault();
        Serial.println("Cell Over Temperature detected");
      }
    }
  }
  TempOpenWire();
}

void MeasureCurrent() {
  uint16_t input_arr[100] = {};
  float paddu_input = 0;
  for (int i = 0; i < 100 ; i++) {
    input_arr[i] = analogRead(A2);
    paddu_input += (1.0 / 100.0) * ((float)input_arr[i]);
  }
  Voltage_Input = ((1.5 * 3.3 * paddu_input) / 4095.0);
  float Sensitivity = 20.0;
  Current = (Voltage_Input - 2.535) * (1000.0 / Sensitivity);
  if (Current > OC_THRESHOLD) {
    Fault();
    Serial.println("Overcurrent detected");
  }
  CSOpenWire();
}

void OverVoltage() {
  for (int cic = 0; cic < total_ic; cic++) {
    byte ovflag0 = ((1 & ((BMS_IC[cic].stat.flags[0]) >> 1))) || (1 & ((BMS_IC[cic].stat.flags[0]) >> 3)) || (1 & ((BMS_IC[cic].stat.flags[0]) >> 5)) || (1 & ((BMS_IC[cic].stat.flags[0]) >> 7));
    byte ovflag1 = ((1 & ((BMS_IC[cic].stat.flags[1]) >> 1))) || (1 & ((BMS_IC[cic].stat.flags[1]) >> 3)) || (1 & ((BMS_IC[cic].stat.flags[1]) >> 5)) || (1 & ((BMS_IC[cic].stat.flags[1]) >> 7));
    byte ovflag2 = ((1 & ((BMS_IC[cic].stat.flags[2]) >> 1))) || (1 & ((BMS_IC[cic].stat.flags[2]) >> 3));
    if (ovflag0 || ovflag1 || ovflag2) {
      Serial.println("Cell Overvoltage Detected");
      Fault();
      break;
    }
  }
}
void UnderVoltage() {
  for (int cic = 0; cic < total_ic; cic++) {
    byte uvflag0 = ((1 & ((BMS_IC[cic].stat.flags[0])))) || (1 & ((BMS_IC[cic].stat.flags[0]) >> 2)) || (1 & ((BMS_IC[cic].stat.flags[0]) >> 4)) || (1 & ((BMS_IC[cic].stat.flags[0]) >> 6));
    byte uvflag1 = ((1 & ((BMS_IC[cic].stat.flags[1])))) || (1 & ((BMS_IC[cic].stat.flags[1]) >> 2)) || (1 & ((BMS_IC[cic].stat.flags[1]) >> 4)) || (1 & ((BMS_IC[cic].stat.flags[1]) >> 6));
    byte uvflag2 = ((1 & ((BMS_IC[cic].stat.flags[2])))) || (1 & ((BMS_IC[cic].stat.flags[2]) >> 2));
    if (uvflag0 || uvflag1 || uvflag2) {
      Serial.println("Cell Undervoltage Detected");
      Fault();
      break;
    }
  }
}
void CellOpenWire() {

}
void TempOpenWire() {

}
void CSOpenWire() {
  if (Voltage_Input < 2.4) {
    Serial.println("Current Sensor Not Connected ");
    Fault();
  }
}
void DisplayMeasured() {
  DisplayVoltage();
  DisplayCurrent();
  DisplayTemp();
}


void DisplayAll() {
  DisplayVoltage();
  DisplayCurrent();
  DisplayTemp();
  DisplayConfig();

}
void DisplayVoltage() {
  Serial.println(" Cell Voltages : ");
  for (int i = 0; i < 10; i++) {
    if (i != 9) {
      for (int cic = 0; cic < total_ic; cic++) {
        Serial.print("    IC");
        Serial.print(cic + 1, DEC);
        Serial.print(" =");
        Serial.print("    C");
        Serial.print(i + 1, DEC);
        Serial.print(" :  ");
        Serial.print(BMS_IC[cic].cells.c_codes[i] * 0.0001, 4);        
      }
      Serial.println("");
    }
    else {
      for (int cic = 0; cic < total_ic; cic++) {
        Serial.print("    IC");
        Serial.print(cic + 1, DEC);
        Serial.print(" =");
        Serial.print("    C");
        Serial.print(i + 1, DEC);
        Serial.print(":  ");
        Serial.print(BMS_IC[cic].cells.c_codes[i] * 0.0001, 4);

      }
      Serial.println("");
    }
  }
  for (int i = 0; i < 4; i++) Serial.println("");
}
void DisplayCurrent() {
  Serial.print(" Cell Current : ");
  Serial.println(Current);
  for (int i = 0; i < 4; i++) Serial.println("");
}
void DisplayTemp() {
  Serial.println(" Cell Temperatures : ");
  for (int i = 0; i < 4; i++) {
    for (int cic = 0; cic < total_ic; cic++) {
      float resistance = ((50000.0 - (BMS_IC[cic].aux.a_codes[i]*1.0)) / 5.0);
      float Temperature = 1.0 / (0.003354 + 0.000257 * log(resistance / 10000.0) + 0.0000026 * log(pow((resistance / 10000.0), 2)) + 0.0000063 * log(pow((resistance / 10000.0) , 3))) - 273.15;
      Serial.print("       T");
      Serial.print(i + 1, DEC);
      Serial.print(":  ");
      Serial.print(Temperature);
    }
    Serial.println(" ");
  }
}
void DisplayConfig() {
  Serial.println(" CONFIGURATION ENABLED ");
  if (Serial.available()) {
    char * input = read_string();
    if (!strcmp(input, "uv")) {
      Serial.println("Enter the new Undervoltage limit : ");
      float uv = read_float();
      UV_THRESHOLD = (uint16_t) (uv * 10000.0);
      EEPROM.write(0,(uint8_t)(uv*10));
    }
    if (!strcmp(input, "ov")) {
      Serial.println("Enter the new Overvoltage limit : ");
      float ov = read_float();
      OV_THRESHOLD = (uint16_t) (ov * 10000.0);
      EEPROM.write(1,(uint8_t)(ov*10));
    }
    if (!strcmp(input, "oc")) {
      Serial.println("Enter the new Overcurrent limit : ");
      float oc = read_float();
      OC_THRESHOLD = oc;
      EEPROM.write(2,OC_THRESHOLD);
    }
    if (!strcmp(input, "ot")) {
      Serial.println("Enter the new Overtemperature limit : ");
      float ot = read_float();
      OT_THRESHOLD = ot;
      EEPROM.write(3,OT_THRESHOLD);
    }
    if (!strcmp(input, "ic")) {
      Serial.println("Enter the number of ICs in daisy chain : ");
      float ic = read_float();
      total_ic = ic;
      
    }
  }
}
void print_selftest_errors(uint8_t adc_reg ,int8_t error)
{
  check_error(error);
  if(adc_reg==1)
  {
    Serial.println("Cell ");
    }
  else if(adc_reg==2)
  {
    Serial.println("Aux ");
    }
  else if(adc_reg==3)
  {
    Serial.println("Stat ");
    }
   Serial.print(error, DEC);
  Serial.println((" : errors detected in Digital Filter and Memory \n"));
}
void check_error(int error) {
  if (error == -1) {
    Fault();
  }
}
void Fault() {
  digitalWrite(MODE_CONTROL, HIGH);
  Serial.println("AMS Fault Triggered");
}
