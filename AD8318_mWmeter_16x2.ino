/**************************************************************************************************************
 * AD8318_mWmeter 10MHz - 10 GHz
 * 
 * Adaptation of Paolo Cravero IK1ZYW - April 2022
 * 
 * Parameters have been adjusted from AD8317 to AD8318.
 * I am using LiquidCrystal_PCF8574 library because there are too many incompatible LiquidCrystal_I2C libraries around!
 * For all the rest refer to the original comment below.
 * 
 * WARNING. I2C address of the display module is around line 142. Change it accordingly!
 */
/**************************************************************************************************************
 * AD8317_mWmeter 10MHz - 10GHz    
 *
 * Design Rob Engberts PA0RWE - November 2018
 *
 * Original design by Joost Breed and functionality by OZ2CPU
 * Based on AD8317 Log Detector of Analog Devices
 * Datasheet: http://www.analog.com/media/en/technical-documentation/evaluation-documentation/AD8317.pdf
 * 
 * V1.0   8-11-2018
 *
 * Functionality:
 * - Menu driven
 *   - Set-up attenuatio of 0, -10, -20, -30, -40 and -50 dB
 *   - RF Powermeter (default)
 *   - Calibration for every band by calculating Slope and Intercept on -10 and -40 dBm
 *   - Read calibration data
 *   - Reset calibration data
 * - 16 character display with on line:  
 *   1 Measured dBm - Attenuation / Band - measured rms voltage in mV
 *   2 Graphic signal strength - measured nW
 * - Bands: HF, 4m, 70cm, 23cm, 13cm, 9cm, 6cm and 3cm  
 * - Band and Attenuation storage after change
 * 
 * Note: Menu functions are in the Menu.ino file. It should be in same folder as the main ino file.
 * External reference = LM4040C201 2.048V
 * 
 * Modifications:
 * RWE: changed for using 16 character I2C display, and based on version 1.2 of the original ino file
 * V1.3   20-09-2021
 * -  Error values corrected.
 * 
 ***************************************************************************************************************/
#include <EEPROM.h>
#include <Bounce2.h>
//#include <LiquidCrystal.h>
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>

/*-----( Declare Constants )-----*/
/*------( Declare objects )------*/

// Forward declaration
int analogReadFast(byte ADCpin, byte prescalerBits=4);

// Constant declarations
//const byte LCD_RS_PIN = 12; 
//const byte LCD_RW_PIN = 11;
//const byte LCD_E_PIN  = 10;
//const byte LCD_DB4_PIN = 9;
//const byte LCD_DB5_PIN = 8;
//const byte LCD_DB6_PIN = 7;
//const byte LCD_DB7_PIN = 6;
    
// Button pins
const byte BTN_MENU   = A2;
const byte BTN_SELECT = A3;
const byte BUTTON_DEBOUNCE_MS = 5;      // Button debounce time in ms.

// Encoder pins
#define encoderPinA 2                   // interrupt pin
#define encoderPinB 4                   // no need to use an interrupt pin here

//volatile long posL = 0;                 // long can hold far more pulses than int; signed to know direction since start.
volatile boolean CW = true;             // direction if false it is CCW
//volatile boolean Left = false;
//volatile boolean Right = false;

// ADC pin for power measurement
const byte  AD_POWER_PIN    = A0;
const float V_REF           = 2.048;    // External reference voltage.
const float AD_RESOLUTION   = 1023.0;   // AD resolution - 1 = 2^10 - 1.
const int   SAMPLE_COUNT    = 15000;    // Amount of samples to take for averaging which will result in sample time of about 500ms.

// Strings in progmem to preserve SRAM
#define FS(x) (__FlashStringHelper*)(x) // Function to get string from Flash memory. String is put in flash with the PROGMEM keyword. To preserve SRAM.
const char str_dB[]  PROGMEM = "dB";
const char str_dBm[] PROGMEM = " dBm ";

// Menus
const byte MENU_ATTENUATION_0dB       = 0;
const byte MENU_ATTENUATION_10dB      = 1;
const byte MENU_ATTENUATION_20dB      = 2;
const byte MENU_ATTENUATION_30dB      = 3;
const byte MENU_ATTENUATION_40dB      = 4;
const byte MENU_ATTENUATION_60dB      = 5;
const byte MENU_RF_METER              = 6;
const byte MENU_CALIBRATION           = 7;
const byte MENU_CALIBRATION_READ      = 8;
const byte MENU_CALIBRATION_RESET     = 9;

// EEPROM Addresses
const short INIT_SCHEMA        = 0x0110;             // Schema to check if eeprom has been initialized for the first time.
const byte INIT_EEPROM_ADDRESS = 0;                  // Address to write schema to.
const byte BAND_EEPROM_ADDRESS = 2;                  // Selected band start address (int = 2 byte).
const byte ATTENUATION_VALUE_EEPROM_ADDRESS   = 5;   // Selected attenuation start address (int = 2 byte).
const byte CALIBRATION_DATA_EEPROM_ADDRESS    = 50;  // Selected calibration data start address


// A frequency band can be selected by the user. 
// The selected frequency determines the typical performance characteristics taken from the datasheet.
// All values below are taken from page 3 and 4 of the datasheet.
const int MIN_INPUT_DBM         = -60;          // Minimum input power. If less then consider as 'not connected'.
const int ABS_MAX_INPUT_DBM     = 7;            // For absolute maximum we take +7 dBm. (More than +12dBm will destroy the IC)
const int BAND                  = 8;            // Amount of Bands.
const char BANDS[BAND][6]       = {"  HF ", "  4m ", " 70cm", " 23cm", " 13cm", "  9cm", "  6cm", "  3cm" };  // List of BANDS we know settings data of.
// >>>>> Note: values below ​​are only used if the calibration has not yet been accomplished!!  <<<<<
const float DBM_AT_0V[BAND]     = {  22,   22,   22,   20.4,   19.6,   19.8,   25,   38 };  // estimated and extrapolated dBm value at 0V (Intercept) for AD8318
const float MV_DB_SLOPE         = 0.0250;       // Slope of AD8318 characteristics in -mV/dBm. Is the same for all BANDS. 
//
// NOTE: The DBM_AT_0V[] values are based on the -10dBm value from the datasheet. 
// i.e. when f=5.8GHz the datasheet notes 590mV at -10dBm. With a slope of -22mV/dBm ==> (590/22m) - 10 = 16.82dBm at 0V.
// During calibration these values are calculated with an input of -10 and -40 dBM and stored in EEPROM

// Error of measured power from -60dBm to 5dBm in 5dB steps per band. 
// Using the value for T=25°C from the datasheet graphs.
                   //  @    -60,  -55,  -50,  -45,  -40,  -35,  -30,  -25,  -20,  -15,  -10,   -5,    0,    5  dBm
const float ERROR_DBM[BAND][14]{                         
                        {  -1.3, -0.6, -0.2,  0.0,  0.0,  0.1,  0.1,  0.2,  0.1,  0.0, -0.2, -0.2,  1.2,  4.0},  // HF   / 900MHz
                        {  -1.3, -0.6, -0.2,  0.0,  0.0,  0.1,  0.1,  0.2,  0.1,  0.0, -0.2, -0.2,  1.2,  4.0},  // 4m   / 900MHz
                        {  -1.3, -0.6, -0.2,  0.0,  0.0,  0.1,  0.1,  0.2,  0.1,  0.0, -0.2, -0.2,  1.2,  4.0},  // 70cm / 900MHz
                        {  -0.8, -0.5, -0.2,  0.0,  0.0,  0.0,  0.0,  0.2,  0.1,  0.0, -0.1,  0.4,  2.0,  4.0},  // 23cm / 1.9GHz
                        {  -0.6, -0.3, -0.2, -0.1,  0.0,  0.0,  0.1,  0.2,  0.2, -0.1, -0.3,  0.2,  2.0,  4.0},  // 13cm / 2.2GHz
                        {   0.2,  0.2, -0.2, -0.2,  0.0,  0.0,  0.1,  0.2,  0.2,  0.0, -0.4, -0.5,  0.8,  4.0},  // 9cm  / 3.6GHz
                        {  -4.0, -0.7, -0.1,  0.0,  0.0, -0.1, -0.1,  0.0,  0.0,  0.0,  0.0, -0.2,  0.1,  1.6},  // 6cm  / 5.8GHz
                        {  -5.0, -2.2, -0.6,  0.0,  0.2,  0.2,  0.2,  0.3,  0.2, -0.3, -0.6, -1.1, -1.6, -1.4}   // 3cm  / 8.0GHz
                      };


// Variable declarations
// LiquidCrystal lcd(LCD_RS_PIN, LCD_RW_PIN, LCD_E_PIN, LCD_DB4_PIN, LCD_DB5_PIN, LCD_DB6_PIN, LCD_DB7_PIN); // LCD driver
LiquidCrystal_PCF8574 lcd(0x3F);
Bounce button_MENU = Bounce();      // Button driver for MENU button
Bounce button_SELECT = Bounce();    // Button driver for SELECT button


// Measured and calculated values
float avg_voltage;              // Current average voltage
float avg_dbm;                  // Current average power in dBm
bool power_overload;            // If true, too much power is put into the device.
bool power_notconnected;        // If true no or too less power is put into the device.
int adc_min_value;              // Used during measurement to keep track of minimum value of samples
int adc_max_value;              // Used during measurement to keep track of maximum value of samples
int avg_min_value;              // Used during measurement to keep track of minimum average value of samples
int avg_max_value;              // Used during measurement to keep track of maximum average value of samples
unsigned long sum = 0;          // Variable to calculate sum of samples
float dBm_at_0V;                // Current dbm at 0V;
float mv_dB_slope;              // Current slope
int attenuation;                // Current attenuation
int old_attn = 0;               // Old attenuation setting

// Menu and display
int current_menu = 0;           // Current selected menu item displayed
byte menumode = 0;              // when in menu = 1
int current_band = 0;           // Current band 0-7
int old_band = 0;               // Old Band setting
//byte cnt = 0;                   // Counter for display Band, Attn and Error

// Calibration data struture
typedef struct
{
  int current_band;             // Current band
  float dBm_at_0V;              // dBm value at 0V
  float mv_dB_slope;            // mV / dBm slope
} calibrationPoint;

const int MAX_CALIBRATION_POINTS = BAND;   // 8 Bands to store

calibrationPoint calibrationData[MAX_CALIBRATION_POINTS];         // Calibration data entered by the user
calibrationPoint mergedCalibrationData[MAX_CALIBRATION_POINTS];   // Calibration data from DBM_AT_0V array merged with user calibration data.

float cal_Pin = 30.0;           // Pin-Pin2 for slope calcualtion
float cal_Vout1;                // Vout1 for slope calculation
float cal_Vout2;                // Vout2 for slope calculation
float cal_dBm_at_0V;            // calculated dBm value at 0V
float cal_mv_dB_slope;          // calculated mV / dBm slope
byte measure = 0;               // Measure sequence 0 and 1


/*****************************************************************************
*                                  S E T U P   
******************************************************************************
* Setup routine
*/
void setup() 
{   
  Serial.begin(9600);
  analogReference(EXTERNAL);    // Set ADC to use external reference
//  lcd.begin(24, 2);             // 24 characters, 2 lines
  lcd.begin(16, 2);
//  lcd.backlight();
//LCDsetgfx();                  // Set-up graphical part
  pinMode(encoderPinA, INPUT);
  attachInterrupt(0, doEncoder, FALLING);
  initButtons();
  getSettingsFromEEPROM();
  mergeCalibrationData();
  displaySplashScreen();
  get_slope_intercept();  
  current_menu = 0;
  menumode = 0;
}


/*****************************************************************************
*                                 M A I N   L O O P   
******************************************************************************
 * The main loop 
 * It is tuned so we have 2 measurements per second.
 * If you want a higher rate then lower SAMPLE_COUNT.
*/
void loop() 
{    
  takeSamples();            // Take all samples (also reads buttons during sampling)
  calculate();              // Calculate values
  readButtons();            // Read buttons
  if (current_band != old_band) 
  {
    mergeCalibrationData();
    get_slope_intercept(); 
    writeSettingsToEEPROM();
    old_band = current_band;
  }
  if (attenuation != old_attn)
  {
    writeSettingsToEEPROM();
    old_attn = attenuation;
  }
 
  // Show measurements or warning
  if (checkInputBounds())   // Check if the measured values are within the device's range. If true returned then overloaded!
  {
    if (menumode == 0) displayMeasurements();       
  }
  else  
    displayOverloadWarning();     // Display overload warning.
  
  readButtons();            // Read buttons   
}

/*****************************************************************************
*                          E N C O D E R   R O U T I N E  
******************************************************************************
*/
void doEncoder()            // Menu select
{
  // determine direction
  CW = (digitalRead(encoderPinA) != digitalRead(encoderPinB)); 
  if (menumode) {
    if (CW) {
      current_menu +=1;
      if (current_menu > 9) current_menu=9;
    }
    else {
      current_menu -=1;
      if (current_menu < 0) current_menu=0;
    }
  }  
  else {
    if (CW) {
      current_band +=1;
      if (current_band > 7) current_band = 7;
    }
    else {
      current_band -=1;
      if (current_band < 0) current_band = 0;
    }
  }    
}

/*****************************************************************************
*                            T A K E   S A M P L E S   
******************************************************************************
 * Take SAMPLE_COUNT of samples
 * sampling SAMPLE_COUNT values with some calculations takes about 500ms.
 * If the program crashes after the spash screen you might have an arduino with another clock speed than 16MHz. 
 * Change the prescaler to another number see: http://www.robotplatform.com/knowledge/ADC/adc_tutorial_2.html
 * Or just use analogRead() instead.
*/
void takeSamples() 
{
  int adc_value;             
  // Reset min, max and sum
  adc_min_value = 1000;
  adc_max_value = -1000;
  sum = 0;  
  for (int i=0; i<SAMPLE_COUNT; i++)
  {    
    adc_value = analogReadFast(AD_POWER_PIN, 4); // Take a sample ~20us        
    // Determine the min, max and sum from current measured data
    if (adc_value < adc_min_value) adc_min_value = adc_value;
    if (adc_value > adc_max_value) adc_max_value = adc_value;    
    sum += adc_value;
    if (i%2000==0) readButtons(); // read buttons between samples, once every 2000 samples.    
  }      
}

/*
 * Function to read ADC within 20us instead of 116us 
 * This function is borrowed from Albert http://www.avdweb.nl/arduino/libraries/fast-10-bit-adc.html
 */
int analogReadFast(byte ADCpin, byte prescalerBits=4)
{ 
  byte ADCSRAoriginal = ADCSRA;
  ADCSRA = (ADCSRA & B11111000) | prescalerBits;
  int adc = analogRead(ADCpin); 
  ADCSRA = ADCSRAoriginal;
  return adc;
}


/*****************************************************************************
*     C A L C U L A T I O N S    A N D   C O N V E R S I O N S 
******************************************************************************
*
* Perform all calculations we need.
*/
void calculate()
{    
  float avg_measurement = sum / SAMPLE_COUNT;                 // Calculate average of current samples.
  avg_voltage = avg_measurement * (V_REF / AD_RESOLUTION);    // Convert to voltage  
  avg_dbm = measuredValueTodBm(avg_measurement);              // Convert average voltage to dBm.  
}

/*
 * Convert the AD converter value to voltage and then dbm
*/
float measuredValueTodBm(int value) 
{
  float voltage = value * (V_REF / AD_RESOLUTION);    
  float dbm = convertVoltageToDbm(voltage);
  return dbm;
}

/*
 * Convert voltage to dBm.
 * According to Figure 8 of the datasheet
 * dBm value at 0V - (measured voltage / calculated slope)
 * The supported range is -50dBm to 0dBm because of high error beyond. 
*/
float convertVoltageToDbm(float voltage) 
{
  return dBm_at_0V - (voltage / mv_dB_slope);
}

/* Convert dBm to mW */
float convertDbmToMilliWatt(float dBm) 
{  
  return pow(10, dBm / 10.0);
}

/* Convert dBm to rms voltage */
float convertdBmToVolt(float dBm) 
{
  float wortel = 0.223607;
  float kwadrt = dBm / 20.0;
  return (wortel * (pow(10.0, kwadrt))) * 1000;
}

/* Round to a certain amount of decimals */
float roundDecimals(float value, int decimals)
{
  float d = pow(10, decimals);
  if (value < 0)
    return ceil((value * d - 0.5)) / d;
  else 
    return floor((value * d + 0.5)) / d;
}

/*
 * Check if the power measured is within the range.
 * Overpower can damage the device.
*/
bool checkInputBounds() 
{
  power_overload = (avg_dbm >= ABS_MAX_INPUT_DBM);
  power_notconnected = (avg_dbm < MIN_INPUT_DBM);  
  return !power_overload;
}


/*****************************************************************************
*                         C A L I B R A T I O N   
******************************************************************************
*
* Reset all calibration settings
*
*/
void deleteCalibrationPoints() {
// Reset all data 

  for (int i=0; i < (MAX_CALIBRATION_POINTS); i++)
    {
      calibrationData[i].current_band = i;
      calibrationData[i].dBm_at_0V = 0.0;  
      calibrationData[i].mv_dB_slope = 0.0;
    }
  writeCalibrationDataToEEPROM();   // Write to EEPROM
  mergeCalibrationData();           // Recalculate merged data
}


/*
 * Calculate Slope and Intercept
 * Calculation is done following the procedure described in the dataset
 * 
 */
void slopeCalculation()
{
  float measuredVoltage = avg_voltage; // Get currently measured avg_voltage
  if (measure == 0) cal_Vout1 = measuredVoltage;
  if (measure == 1) {
    cal_Vout2 = measuredVoltage;
    cal_mv_dB_slope = ((cal_Vout1-cal_Vout2) / cal_Pin) * -1.0;
    cal_dBm_at_0V = -10 + (cal_Vout1 / mv_dB_slope);
    Serial.println("Calibration data:");
    Serial.print(cal_mv_dB_slope, 5);
    Serial.print(" : ");
    Serial.print(cal_dBm_at_0V, 4);
    Serial.print(" : ");
    Serial.print(cal_Vout1, 4);
    Serial.print(" : ");
    Serial.println(cal_Vout2, 4);
  }
}


/*
 * Save calculated calibration data to EEPROM
*/
void saveCalibrationData()
{
  calibrationData[current_band].current_band = current_band;
  calibrationData[current_band].dBm_at_0V = cal_dBm_at_0V;
  calibrationData[current_band].mv_dB_slope = cal_mv_dB_slope;
  // Save updated data to EEPROM
  writeCalibrationDataToEEPROM();
}

/*
 * Merges calibrationData and DBM_AT_0V into one array of type calibrationPoint 
 * and replace the default dBm_at_0V value by the calibration value
*/
void mergeCalibrationData()
{
// Add hardcoded calibration data.
  for (int i=0; i<MAX_CALIBRATION_POINTS; i++)
  {
    mergedCalibrationData[i].current_band = i;
    mergedCalibrationData[i].dBm_at_0V = DBM_AT_0V[i];
    mergedCalibrationData[i].mv_dB_slope = MV_DB_SLOPE;
  }

// Fill merged array with calibration data (if not 0.00) of the user.
  for (int i=0; i<MAX_CALIBRATION_POINTS; i++)
  {    
    mergedCalibrationData[i].current_band =  calibrationData[i].current_band;
    if (calibrationData[i].dBm_at_0V != 0.0)
    {
      mergedCalibrationData[i].dBm_at_0V =  calibrationData[i].dBm_at_0V;
      mergedCalibrationData[i].mv_dB_slope = calibrationData[i].mv_dB_slope;
    }    
  }

  
// Debug stuff

  Serial.println("Merged data:");
  for (int i=0; i<MAX_CALIBRATION_POINTS; i++)
  {
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(mergedCalibrationData[i].current_band);
    Serial.print(" : ");
    Serial.print(mergedCalibrationData[i].dBm_at_0V, 4);
    Serial.print(" : ");
    Serial.println(mergedCalibrationData[i].mv_dB_slope, 4);
  }
}

/* 
 * Get CalibrationData from merge array values 
 */
void get_slope_intercept()  
{  
  int minIdx = 0;
  int amountItems = MAX_CALIBRATION_POINTS;
  for (minIdx; minIdx < amountItems; minIdx++)
  {
      if (current_band == mergedCalibrationData[minIdx].current_band)
      {
        dBm_at_0V = mergedCalibrationData[minIdx].dBm_at_0V;
        mv_dB_slope = mergedCalibrationData[minIdx].mv_dB_slope;
      }
  }  
}


/*****************************************************************************
*     F O R M A T T I N G   A N D   O U T P U T   T O    D I S P L A Y   
******************************************************************************
*
* Print power in uW, mW or W.
*/
byte printPowerWatts(float mw) 
{
  byte chars = 0;
  if (mw < 0.001) // If less then 10uW display in nW
  {
    chars = printFormattedNumber(mw * 1000000.0, 1, 0, false, false);
    chars += lcd.print(F("nW"));        
  }
  else if (mw < 0.01) // If less then 10uW display in 0.1 uW resolution
  {
    chars = printFormattedNumber(mw * 1000.0, 1, 1, false, false);
    chars += lcd.print(F("uW"));        
  }
  else if (mw < 1.0) // If less then 1mW display in uW
  {
    chars = printFormattedNumber(mw * 1000.0, 1, 0, false, false);
    chars += lcd.print(F("uW"));        
  }
  else if (mw >= 1000.0) // If more than 1000mW display in W
  {
    chars = printFormattedNumber(mw / 1000.0, 1, 1, false, false);
    chars += lcd.print(F("W"));        
  }
  else
  {
    chars = printFormattedNumber(mw, 1, 1, false, false);
    chars += lcd.print(F("mW"));        
  }
  return chars;
}

/*
* Print voltage in uV, mV or V.
*/
byte printVoltage(float mV) 
{
  byte chars = 0;
  if (mV < 0.001) // If less then 10uV display in nV
  {
    chars = printFormattedNumber(mV * 1000000.0, 1, 0, false, false);
    chars += lcd.print(F("nV"));        
  }
  else if (mV < 0.01) // If less then 10uV display in 0.1 uV resolution
  {
    chars = printFormattedNumber(mV * 1000.0, 1, 1, false, false);
    chars += lcd.print(F("uV"));        
  }
  else if (mV < 1.0) // If less then 1mV display in uV
  {
    chars = printFormattedNumber(mV * 1000.0, 1, 0, false, false);
    chars += lcd.print(F("uV"));        
  }
  else if (mV >= 1000.0) // If more than 1000mV display in V
  {
    chars = printFormattedNumber(mV / 1000.0, 1, 1, false, false);
    chars += lcd.print(F("V"));        
  }
  else
  {
    chars = printFormattedNumber(mV, 1, 1, false, false);
    chars += lcd.print(F("mV"));        
  }
  return chars;
}


/*
 * Fillout a complete display line with spaces.
*/
//void filloutLine(byte charsWritten) 
//{
 // for (byte i=0; i < 24 - charsWritten;i++)
     // lcd.print(" ");    
//}

/*
 * Print a number with a specific amount of digits and decimals on the display.
 * The function does not round the number.
 * It is also possible to print a space when there is no negative sign.
 * number: Number to display
 * minAmountDigits: The minimum amount of digits to display. If number is shorter then '0's will be printed before the number.
 * amountDecimals: The amount of decimals to display.
 * fillout: If the amount of decimals is shorter then the desired amount, spaces will be used to get the same string length.
*/
byte printFormattedNumber(float number, byte minAmountDigits, byte amountDecimals, bool reserveMinus, bool fillout)
{
  byte chars = 0; // Number of chars written.
  
  // If no minus sign, write a space
  if (reserveMinus && number >= 0)
    chars += lcd.print(" ");
  else if (number < 0) {
    chars += lcd.print("-");
    number *= -1;
  }
    
  //When current value has less digits then wanted add extra zero's
  int digitPart = (int)number;
  byte digitCount = getAmountDigits(digitPart);
  if (digitCount < minAmountDigits)
  {
    for (byte i=0; i < minAmountDigits - digitCount; i++)
      chars += lcd.print("0");
  }
  chars += lcd.print(number, amountDecimals); 
  if (fillout) 
  {
    // Fill missing chars according to desired length
    byte totalCharsWanted = minAmountDigits + amountDecimals;
    if (reserveMinus)
      totalCharsWanted++;    
    for (byte i=0; i < totalCharsWanted - chars; i++)
      chars += lcd.print(" ");   
  }  
  return chars; // return the amount of chars written to the display
}


/*
 * Get amount of digits of an integer.
 * Simple but fast
*/
byte getAmountDigits(int n)
{
    n = abs(n);
    if (n < 10) return 1;
    if (n < 100) return 2;
    if (n < 1000) return 3;
    if (n < 10000) return 4;    
}

/*****************************************************************************
*                          L O A D   G R A P H I C S   
******************************************************************************/
void LCDsetgfx()                             // user defined graphics for the bar-graf
{
  byte char1[8] = {0x00,0x00,0x10,0x10,0x10,0x10,0x10,0x10};
  byte char2[8] = {0x00,0x00,0x18,0x18,0x18,0x18,0x18,0x18};
  byte char3[8] = {0x00,0x00,0x1c,0x1c,0x1c,0x1c,0x1c,0x1c};
  byte char4[8] = {0x00,0x00,0x1e,0x1e,0x1e,0x1e,0x1e,0x1e};
  byte char5[8] = {0x00,0x00,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f};
  
  lcd.createChar(0, char1);
  // create a new character
  lcd.createChar(1, char2);
  // create a new character
  lcd.createChar(2, char3);
  // create a new character
  lcd.createChar(3, char4);
  // create a new character
  lcd.createChar(4, char5);
}



/*****************************************************************************
*             C A L C U L A T E   M E A S U R E M N T   E R R O R  
******************************************************************************
*
 * Calculate current measurement error.
 * The error varies a lot by temperature. So we do it the simple way
 * and take the closest of an average error.
*/
float currentError() 
{
  int roundedDbm = (int)(round((avg_dbm - attenuation) / 5.0) * 5.0);   // Round to nearest 5 dB
  int idx = (roundedDbm + 60) / 5;                                      // Get array index. 0 = -60dBm
  if (idx < 0)                                                          // Check bounds
    idx = 0;
  else if (idx >= (sizeof(ERROR_DBM[current_band]) / sizeof(float))) 
    idx = (sizeof(ERROR_DBM[current_band]) / sizeof(float)) - 1;        // determnine array location
  return ERROR_DBM[current_band][idx];
}


/*****************************************************************************
*                    I N I T I A L I Z E   B U T T O N S   
******************************************************************************
 * Initialize the buttons
*/
void initButtons() 
{
  // Set button pins as input.  
  pinMode(BTN_MENU ,INPUT);
  pinMode(BTN_SELECT ,INPUT);
  // Attach pins to Bounce
  button_MENU.attach(BTN_MENU);
  button_SELECT.attach(BTN_SELECT);
  // Set bounce interval
  button_MENU.interval(BUTTON_DEBOUNCE_MS);
  button_SELECT.interval(BUTTON_DEBOUNCE_MS);
}


/*****************************************************************************
*                  E E P R O M   R E A D   A N D   W R I T E   
******************************************************************************
 * Read settings from EEPROM
*/
void getSettingsFromEEPROM() 
{ 
  // Check if EEPROM has been initialized.
  short schema = 0;
  EEPROM_readAnything(INIT_EEPROM_ADDRESS, schema);  
  if (schema != INIT_SCHEMA)   // Compare with expected schema   
    initEEPROM();   // Initialize EEPROM with default value

  // Read values from EEPROM
  EEPROM_readAnything(BAND_EEPROM_ADDRESS, current_band);  
  if (current_band > BAND) current_band = 0;
  EEPROM_readAnything(ATTENUATION_VALUE_EEPROM_ADDRESS, attenuation); 
  if (attenuation > 0 || attenuation < -50) attenuation = 0;
  getCalibrationDataFromEEPROM();
}

void getCalibrationDataFromEEPROM()
{
  EEPROM_readAnything(CALIBRATION_DATA_EEPROM_ADDRESS, calibrationData);   
}

/*
 * Initialize the EEPROM with defaults only first time
*/
void initEEPROM()
{
  // Set default values
  current_band = 0;
  attenuation  = 0;
  writeSettingsToEEPROM();
  
  // Clear calibration
  for (int i=0; i < MAX_CALIBRATION_POINTS; i++)
  {
    calibrationData[i].current_band = 0;
    calibrationData[i].dBm_at_0V = 0;
  }  
  writeCalibrationDataToEEPROM();
  
  // Write schema so we can skipt this function the next time.
  EEPROM_writeAnything(INIT_EEPROM_ADDRESS, INIT_SCHEMA); // Mark EEPROM as initialized
}

/*
 * Write settings to EEPROM
*/
void writeSettingsToEEPROM() 
{
  EEPROM_writeAnything(BAND_EEPROM_ADDRESS, current_band);                      //Write selected band to EEPROM  
  EEPROM_writeAnything(ATTENUATION_VALUE_EEPROM_ADDRESS, attenuation);          //Write selected attenuation to EEPROM  
}

/*
 * Write calibration data to EEPROM
*/
void writeCalibrationDataToEEPROM()
{
  EEPROM_writeAnything(CALIBRATION_DATA_EEPROM_ADDRESS, calibrationData);
}

/*
 * Store any type in the EEPROM
 */
template <class T> int EEPROM_writeAnything(int adr, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(adr++, *p++);
    return i;
}

/*
 * Read any type in the EEPROM
 */
template <class T> int EEPROM_readAnything(int adr, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(adr++);
    return i;
}
