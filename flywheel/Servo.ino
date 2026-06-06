// Function to control speed and direction
void setServoSpeed(int speed) {
  // Map speed (-100 to 100) to pulse width (1000 to 2000 µs)
  int pulseWidth = map(speed, -100, 100, 1000, 2000);
  
  // Convert pulse width to duty cycle (0-255 for Pico's analogWrite)
  int dutyCycle = map(pulseWidth, 1000, 2000, 26, 51);  // ~5% to ~10% duty cycle

  analogWrite(8, dutyCycle);  // Set PWM output
}


void Servosetup() {
  printOLED("Testing servo...");
  pinMode(8, OUTPUT);
  analogWriteFreq(50);  // Set PWM frequency to 50 Hz (20ms period)
  analogWriteRange(255);
  pinMode(ServoStatusLED, OUTPUT);
  digitalWrite(ServoStatusLED, HIGH);
  delay(100);



  setServoSpeed(100);  // Full speed forward
  delay(250);

  pinMode(8, OUTPUT);

  setServoSpeed(50);   // Half speed forward
  delay(250);


  setServoSpeed(0);    // Stop
  delay(250);
  
  
  setServoSpeed(-50);  // Half speed reverse
  delay(250);


  setServoSpeed(-100); // Full speed reverse
  delay(250);  

  setServoSpeed(0);    // Stop
  delay(2000);

  digitalWrite(ServoStatusLED, LOW);
  delay(100);

  while (calibration[3] != 3) {
    BNOupdate();
    printOLED("MAG: " + String(calibration[3]));
    digitalWrite(ERR_LED_PIN, HIGH);
    }
    digitalWrite(ERR_LED_PIN, LOW);

    digitalWrite(ERR_LED_PIN, HIGH);
    digitalWrite(ServoStatusLED, HIGH);
    delay(500);
    digitalWrite(ERR_LED_PIN, LOW);
    digitalWrite(ServoStatusLED, LOW);


}

float calculateError(float current, float target) {
  float difference = target - current;
  if (difference < -180) {
    difference += 360;
  }
  else if (difference > 180) {
    difference -= 360;
  }

  if (abs(difference) <= 2.5) {
    digitalWrite(ServoStatusLED, HIGH);
  } 

  else if (abs(difference) > 2.5) {
    digitalWrite(ServoStatusLED, LOW);
  }

  return difference;
}

float PID(float error) {
  float currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;  // Convert to seconds
  /*
  if (error > 0) {
    error -= 2.5;
  }
  else {
    error += 2.5;
  }
  */
  // Proportional term
  //float proportional = KP * error;
  /*
  if (abs(error) < 1) {
    KP = 2.0;
  }

  else if (abs(error) < 5) {
    KP = 1.0;
  } 
  else {
    KP = 1.5;
  }
  */
  proportional = KP * error;

  

  prevProp = proportional;
  
  // Integral term (with anti-windup)
  //float integral = 0;
  /*
  if (abs(error) < 5.0) {
    KI = 2.5;
  }

  else {
    KI = 1.5;
  }
  */
  integral += KI * error * deltaTime;
  integral = constrain(integral, -maxServoSpeed, maxServoSpeed);
  /*
  if (previousError * error < 0) {
    integral *= 0.99;
  }

  if (abs(error) < tolerance)       //"bleeds" the integral as to slow down around target
  {
      integral *= 0.99;
  }
  */
  //integral = 0;
  // Derivative term
  //float derivative = 0;
  if (deltaTime > 0) {
    derivative = KD * (error - previousError) / deltaTime;
  }
  


  // Calculate total output
  float output = round(proportional + integral + derivative); //added rounding, constrain seems to always round DOWN, such as in cases that end in .9
  // Update variables for next iteration
  previousError = error;
  previousTime = currentTime;
  
  return output;
}

int mapPIDToServo(float pidOutput) {
  // Constrain PID output to servo range

  

  pidOutput = constrain(pidOutput, -maxServoSpeed, maxServoSpeed);
  
  // Map PID output to servo command
  int servoCommand = servoOff + pidOutput;
  
  return servoCommand;
}


void Servoupdate() {
  currentHeading = orientation[0];
  orientationError = calculateError(currentHeading, targetOrient);

  pidOutput = PID(orientationError);

  servoCommand = mapPIDToServo(pidOutput);
  
  if (servoCommand < servoOff -0 && servoCommand > -minServoSpeed) {    //changed from -5 to 0
    servoCommand = -minServoSpeed;
  }

  else if (servoCommand > servoOff +0 && servoCommand < minServoSpeed) {   //changed from +5 to 0
    servoCommand = minServoSpeed;
  }
  //printOLED(String(servoCommand), true);
  
  if (abs(servoCommand) >= 65) {
    saturationCounter++;
    if (saturationCounter >= 150 && saturationTimer == 0) {
      saturationTimer = millis();
    }
  }

  else if (abs(servoCommand) < 65 && saturationCounter <= 150) {
    saturationCounter = 0;
    saturationTimer = 0;
  }


  if ((millis() >= (saturationTimer + 5000)) && (saturationCounter >= 150)) {
    saturationCounter = 0;
    saturationTimer = 0;
  }

  else if (saturationCounter >= 150) {
    servoCommand = 0;
  }

  setServoSpeed(-servoCommand); 

  

}