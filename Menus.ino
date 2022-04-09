
/*****************************************************************************
*          R E A D   B U T T O N S   A N D   D I S P L A Y    M E N U   
******************************************************************************
 * Read the buttons and display the correct menu if button is clicked.
 * Each item in the switch statement is a menu. For each menu the buttons actions are handled.
 *  * This Menu file is for the 16 character display only !
*/
void readButtons() 
{
// Get button states  
  bool menu_pressed = (button_MENU.update() && button_MENU.rose());  
  bool select_pressed = (button_SELECT.update() && button_SELECT.rose());
  byte chars = 0;

  if (menu_pressed)
  {
    if (menumode == 0)
    {
      menumode = 1;
    }
    else
    {
      menumode = 0;
      displayMeasurements();
    }
  }

  if (menumode)
  {
  lcd.home();
  switch(current_menu)
    {
    case MENU_ATTENUATION_0dB:      // 0
      if (select_pressed){
        attenuation = 0;
        displayMeasurements();
        menumode = 0;
      }
      else {
        attn_menu_text(0);
        select_menu_text();
      }  
    break;
//*****************************************************
    case MENU_ATTENUATION_10dB:     // 1
      if (select_pressed){
        attenuation = -10;
        displayMeasurements();
        menumode = 0;
      }
      else {
        attn_menu_text(-10);
        select_menu_text();
      }  
    break;
//*****************************************************
    case MENU_ATTENUATION_20dB:     // 2
      if (select_pressed){
        attenuation = -20;
        displayMeasurements();
        menumode = 0;
      }
      else {
        attn_menu_text(-20);
        select_menu_text();
      }  
    break;
//*****************************************************
    case MENU_ATTENUATION_30dB:     // 3
      if (select_pressed){
        attenuation = -30;
        displayMeasurements();
        menumode = 0;
      }
      else {
        attn_menu_text(-30);
        select_menu_text();
      }  
    break;
//*****************************************************
    case MENU_ATTENUATION_40dB:     // 4
      if (select_pressed){
        attenuation = -40;
        displayMeasurements();
        menumode = 0;
      }
      else {
        attn_menu_text(-40);
        select_menu_text();
      }  
    break;
//*****************************************************
    case MENU_ATTENUATION_60dB:     // 5
      if (select_pressed){
        attenuation = -60;
        displayMeasurements();
        menumode = 0;
      }
      else {
        attn_menu_text(-60);
        select_menu_text();
      }  
    break;
//*****************************************************
    case MENU_RF_METER:             // 6
      if (select_pressed){
        menumode = 0;
      }
      else {
        lcd.home();
        chars = lcd.print(F("Menu:")); chars += lcd.print(current_menu);
        chars += lcd.print(F(" RF Power ")); 
        select_menu_text();
      }
    break;
//*****************************************************
    case MENU_CALIBRATION:          // 7
      if (select_pressed){
        lcd.home();
        float actualPowerdBm = roundDecimals(avg_dbm - attenuation, 1);
        if ((measure == 0 && (actualPowerdBm < -20 || actualPowerdBm > 0)) || (measure == 1 && (actualPowerdBm < -50 || actualPowerdBm > -30)))
        {
          lcd.print(F("Not 0 and -20dB "));
          lcd.setCursor(0,1);
          lcd.print(F("or -30 and -50dB"));
          delay(2000);
          menumode = 0;
          measure = 0;
          displayMeasurements();
        }  
        else
        {
          slopeCalculation();
          delay(1000);
          if (measure == 1)
          {  
            saveCalibrationData();
            mergeCalibrationData();
            get_slope_intercept();
            displayCalibrationSavedNotification();
            menumode = 0;
            measure = 0;
            displayMeasurements();
          }
          else if (measure == 0) measure = 1;
        }
      }
      else {
        lcd.home();
        chars = lcd.print(F("Menu:")); chars += lcd.print(current_menu);
        if (measure == 0) {
          chars += lcd.print(F(" Cal-10dBm")); chars += lcd.print(BANDS[current_band]);
        } 
        else if (measure == 1) {
          chars += lcd.print(F(" Cal-40dBm")); chars += lcd.print(BANDS[current_band]);
        }
        select_menu_text();
      }
    break;
//*****************************************************
    case MENU_CALIBRATION_READ:     // 8
      if (select_pressed){
        lcd.home();             // Line 1
        for (int i=0; i<2; i++)
        {
          lcd.print(F("  "));
          lcd.print(mergedCalibrationData[i].dBm_at_0V);
          lcd.print(F(" "));
        }
        lcd.setCursor(0,1);     // Line 2
        for (int i=2; i<4; i++)
        {
          lcd.print(F("  "));
          lcd.print(mergedCalibrationData[i].dBm_at_0V);
          lcd.print(F(" "));
        }
        
        while(!(button_SELECT.update() && button_SELECT.rose())) {   // Wait on select button
        delay(50);
        }

        lcd.home();             // Line 1
        for (int i=4; i<6; i++)
        {
          lcd.print(F("  "));
          lcd.print(mergedCalibrationData[i].dBm_at_0V);
          lcd.print(F(" "));
        }
        lcd.setCursor(0,1);     // Line 2
        for (int i=6; i<8; i++)
        {
          lcd.print(F("  "));
          lcd.print(mergedCalibrationData[i].dBm_at_0V);
          lcd.print(F(" "));
        }

        select_pressed = false;
        
        while(!(button_SELECT.update() && button_SELECT.rose())) {   // Wait on select button
        delay(50);
        }
        
        menumode = 0;
        displayMeasurements();
      }
      else {
        lcd.home();
        chars = lcd.print(F("Menu:")); chars += lcd.print(current_menu);
        chars += lcd.print(F(" Read Cal.")); 
        select_menu_text();
      }
    break;
//*****************************************************
    case MENU_CALIBRATION_RESET:    // 9
      if (select_pressed){
        lcd.home();
        deleteCalibrationPoints();
        displayCalibrateDeletedMenuNotification();
        menumode = 0;
        displayMeasurements();
      }
      else {
        lcd.home();
        chars = lcd.print(F("Menu:")); chars += lcd.print(current_menu);
        chars += lcd.print(F(" Zero Cal.")); 
        select_menu_text();
      }
    break;    
    }  // end of switch

  bool menu_pressed = false;  
  bool select_pressed = false;                
  } 
}

void attn_menu_text(int attn)
{
  byte chars = 0;
  chars = lcd.print(F("Menu:")); chars += lcd.print(current_menu);
  chars += lcd.print(F(" Attn= ")); chars += lcd.print(attn);
  if (current_menu == 0) chars += lcd.print(FS(str_dB));
}

void select_menu_text()
{
  lcd.setCursor(0, 1);        // goto next line
  lcd.print(F("Select= Activate"));
}

/*
 * Wait for x ms but read buttons meanwhile
*/
void readButtonsAndWait(int waitMs)
{
  unsigned long t = millis();
  while (millis() - t < waitMs)
    readButtons();
}

int blinkCnt = 0;
/*
 * Display warning message
*/
void displayOverloadWarning() 
{
  blinkCnt++;
  if (blinkCnt % 2 == 0) {
    lcd.home();  
    lcd.print(F("OVERLOAD WARNING"));         
    lcd.setCursor(0, 1); // goto next line
    lcd.print(F("DISCONNECT NOW! "));        
    lcd.display();
  }
  else
    lcd.noDisplay();
}

/*****************************************************************************
*                      D I S P L A Y    M E A S U R E M E N T S   
******************************************************************************
*  
* Display the settings and primary measurements
* Frequency, attenuation and power in dBm and mW
*/
void displayMeasurements() 
{
  byte chars = 0;
  lcd.home();
  
// Show power measurement in dBm.
  float actualPowerdBm = roundDecimals(avg_dbm - attenuation, 1);
  Serial.print(avg_dbm - attenuation);Serial.print("--");Serial.println(attenuation);
  printFormattedNumber(actualPowerdBm, 1, 1, true, true);    
  lcd.print(FS(str_dBm));  


  Serial.println(current_band);
  // Display Band on first line
  lcd.setCursor(10, 0);
  lcd.print(BANDS[current_band]);   
  lcd.print(" ");       // filler
   


  //cnt++;
  //if (cnt > 10) cnt = 0; // reset loop

  //lcd.print(F("        ")); 
  
  //float actualVoltage = convertdBmToVolt(actualPowerdBm);
  //char pos = printVoltage(actualVoltage);
  //lcd.setCursor(24-pos, 0);                   // to get fixed position
  //printVoltage(actualVoltage);    

  
// Line 2
  lcd.setCursor(0, 1); // goto next line     
  if (power_notconnected)     
    lcd.print(F(" INPUT TOO LOW! "));           // Show not connected
  else
  {           
// Show power measurement in watt.
    float actualPowerMw = convertDbmToMilliWatt(actualPowerdBm);
    char pos = printPowerWatts(actualPowerMw);    
// power in mW on second line
    lcd.print(F("       "));                   // clear line
    lcd.print(printPowerWatts(actualPowerMw));

    lcd.setCursor(6, 1);                        // and Attenuation
    lcd.print(F("Attn "));   
    if (attenuation == 0) chars += lcd.print(F("  ")); 
    printFormattedNumber(attenuation, 1, 0, false, true);
    lcd.print(FS(str_dB));      
//    lcd.print(F("  ")); 
//    printFormattedNumber(currentError(), 1, 1, false, false);    
  }    
}


/*****************************************************************************
*            D I S P L A Y    C A L I B R A T I O N S    N O T E S   
******************************************************************************
*
* Display calibration point deleted notification
*/
void displayCalibrateDeletedMenuNotification() {
  lcd.home();  
  lcd.print(F("   Calibration  "));    
  lcd.setCursor(0, 1);  
  lcd.print(F("    deleted!    "));    
  delay(2000);
}

/*
 * Display calibration saved notification
*/
void displayCalibrationSavedNotification()
{
  lcd.home();
  lcd.print(F("Slope & Intercpt"));
  lcd.setCursor(0, 1);
  lcd.print(F("calculated saved"));
  delay(2000);  
}

/*****************************************************************************
*                              S P L A S H   S C R E E N   
******************************************************************************
 * Display the splash screen
*/
void displaySplashScreen() 
{
  lcd.home();
  lcd.setBacklight(1);
  lcd.print(F(" RF PWR Mtr v1.3"));
  lcd.setCursor(0, 1);
  lcd.print(F("8318  1MHz-10GHz"));
  delay(2000);
  lcd.home();
  lcd.print(F("Max. input power"));
  lcd.setCursor(0, 1);
  lcd.print(F("  +7dBm / 5mW   "));  
  delay(2000);
}
