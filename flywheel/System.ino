/*
_____________________________________________________________
Code for the entire system of the Pi Pico W.
Code by: Radhakrishna Vojjala
Date of last modification: 21 Apr 2026
_____________________________________________________________
This file contains the Setup and Update functions for the entire system. The code is meant to be called in setup() and loop().
*/

// System wide setup function.

void systemSetup() {

  Serial.begin(SERIAL_BAUD);

  Wire.begin(); // default I2C clock
  Wire1.setSCL(I2C_1_SCL);
  Wire1.setSDA(I2C_1_SDA);
  Wire1.begin(); // default I2C clock

  beginOLED(); 

  Serial.println("Initialising....");
  printOLED("Initialising....", true);
  pinMode(ERR_LED_PIN, OUTPUT); // red LED
  pinMode(LOOP_LED_PIN, OUTPUT); // yellow LED
  //pinMode(LOCK_LED_PIN, OUTPUT); // blue LED

  // LED test
  digitalWrite(ERR_LED_PIN, HIGH);
  delay(100);
  digitalWrite(LOOP_LED_PIN, HIGH);
  delay(100);
  //digitalWrite(LOCK_LED_PIN, HIGH);
  delay(100);
  digitalWrite(ERR_LED_PIN, LOW);
  delay(100);
  digitalWrite(LOOP_LED_PIN, LOW);
  delay(100);
  //digitalWrite(LOCK_LED_PIN, LOW);

  if (bno.begin()){
    Serial.println("BNO Online!");
    printOLED("BNO Online!", true);
  }
  else {
    Serial.println("BNO Offline! Check wiring.");
    printOLED("BNO Offline!\nCheck wiring.", true);
    digitalWrite(ERR_LED_PIN, HIGH);
    delay(500);
    digitalWrite(ERR_LED_PIN, LOW);
    error = true;
  }

  /*
  if (MSsetup()){
    Serial.println("MS5611 Online!");
    printOLED("MS5611 Online!", true);
  }
  else {
    Serial.println("MS5611 Offline! Check wiring.");
    printOLED("MS5611 Offline!\nCheck wiring.", true);
    digitalWrite(ERR_LED_PIN, HIGH);
    delay(500);
    digitalWrite(ERR_LED_PIN, LOW);
    error = true;
  }
  */
  inTher.begin(10);
  outTher.begin(10);
  inTher.update();
  outTher.update();
  inStatus = inTher.updateStatus();
  outStatus = outTher.updateStatus();

  if (inStatus){
    Serial.println("Internal Thermistor Connected!");
    printOLED("Internal Thermistor\nConnected!", true);
  }
  else {
    Serial.println("Internal Thermistor Offline. Check wiring.");
    printOLED("Internal Thermistor\nOffline.\nCheck wiring.", true);
    digitalWrite(ERR_LED_PIN, HIGH);
    delay(500);
    digitalWrite(ERR_LED_PIN, LOW);
    error = true;
  }
  
  if (outStatus){
    Serial.println("External Thermistor Connected!");
    printOLED("External Thermistor\nConnected!", true);
  }
  else {
    Serial.println("External Thermistor Offline. Check wiring.");
    printOLED("External Thermistor\nOffline.\nCheck wiring.", true);
    digitalWrite(ERR_LED_PIN, HIGH);
    delay(500);
    digitalWrite(ERR_LED_PIN, LOW);
    error = true;
  }

  //GPSsetup();

  BNOsetup();
  
  Servosetup();

  /*
     Add setup code for additional sensors here
  */

  SDsetup(dataFilename, dataFileN1, dataFileN2);
  logData(header, dataFilename);

  loopTime = 1000 / DATA_RATE;

  Serial.println("Setup Finished");
  printOLED("Setup Finished", true);
  Serial.println(header);

  if (!SDstatus){
    error = true;
  }
  if (error){
    digitalWrite(ERR_LED_PIN, HIGH);
  }
}

// System wide update function. Updates all sensors.

void systemUpdate(){

  // updating timers

  nowTimeMS = millis();
  digitalWrite(LOOP_LED_PIN, LOW);
  nowTimeS = nowTimeMS / 1000.0;
  nowTimeMin = nowTimeS / 60;
  freq = 1.0/((nowTimeMS-prevTime)/1000.0);
  HHMMSS = timeToHhmmss(nowTimeMS);
  if (flightStarted){
    flightTimeMS = nowTimeMS - flightOffset;
    flightTimeHHMMSS = timeToHhmmss(flightTimeMS);
  }

  Volt = analogRead(A3)*3*3.3/(1023); // battery voltage

  // updating sensors

  //GPSupdate();
  //MSupdate();
  BNOupdate();

  inTher.update();
  inTempF = inTher.getTempF();
  inTempC = inTher.getTempC();
  inTempRK = ((inTempC + 273.15) / 180.0) * PI;
  outTher.update();
  outTempF = outTher.getTempF();
  outTempC = outTher.getTempC();
  outTempRK = ((outTempC + 273.15) / 180.0) * PI;

  /*
    Addtional sensor update code here.
  */

  Servoupdate();

  

  digitalWrite(LOOP_LED_PIN, HIGH);
}

// Function to convert timer to HHMMSS format 

String timeToHhmmss(int milli) {

  int timerS = milli / 1000;
  int hours = timerS / 3600;
  int hoursRem = timerS % 3600;
  int mins = hoursRem / 60;
  int secs = hoursRem % 60;
  char timeStr[20];
  sprintf(timeStr, "%02d:%02d:%02d", hours, mins, secs);
  return String(timeStr);
}