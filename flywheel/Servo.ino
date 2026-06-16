// ─────────────────────────────────────────────
//  Flywheel.ino  —  Reaction-wheel stabilizer
//  Uses IMU heading (orientation[0]) to hold the
//  box at targetOrient via a TB6612FNG motor driver
// ─────────────────────────────────────────────

// ── Pin definitions ──────────────────────────
const int STBY  = 14;
const int AIN1 = 12;
const int AIN2 = 13;
const int PWMA  = 15;




const int maxFlywheelSpeed = 255;   // PWM ceiling (0–255)
const int minFlywheelSpeed = 5;    // Deadband — below this the motor won't move



// ── Saturation protection ─────────────────────
// If the flywheel is pegged at full speed for too long,
// something is wrong — cut power to avoid overheating.

const int   SAT_COUNT_LIMIT = 150;   // Consecutive full-speed updates before timer starts
const float SAT_TIME_LIMIT  = 5000;  // ms at full speed before cutting out

// ── Live values (read by main loop / OLED etc.) ──



// ─────────────────────────────────────────────
//  setMotorSpeed(int speed)
//  speed: -255 (full reverse) … 0 (stop) … 255 (full forward)
// ─────────────────────────────────────────────
void setMotorSpeed(int speed) {
  speed = constrain(speed, -maxFlywheelSpeed, maxFlywheelSpeed);

  if (speed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (speed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    // Active brake: both LOW lets the motor coast;
    // set both HIGH for a harder brake if preferred.
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }

  analogWrite(PWMA, abs(speed));
}


// ─────────────────────────────────────────────
//  FlywheelSetup()
//  Call once from setup().  Runs a short ramp
//  test so you can confirm the motor moves, then
//  waits for the IMU magnetometer to calibrate.
// ─────────────────────────────────────────────
void FlywheelSetup() {
  printOLED("Testing flywheel...");

  pinMode(PWMA,    OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY,    OUTPUT);

  digitalWrite(STBY, HIGH);   // Take driver out of standby

  pinMode(ServoStatusLED, OUTPUT);
  digitalWrite(ServoStatusLED, HIGH);

  // Ramp test — verify motor spins both directions
  setMotorSpeed(255);   delay(250);
  setMotorSpeed(128);   delay(250);
  setMotorSpeed(0);     delay(250);
  setMotorSpeed(-128);  delay(250);
  setMotorSpeed(-255);  delay(250);
  setMotorSpeed(0);     delay(2000);

  digitalWrite(ServoStatusLED, LOW);
  delay(100);

  // Wait for magnetometer calibration (same pattern as servo code)
unsigned long calStartTime = millis();
while (calibration[3] != 3) {
    BNOupdate();
    printOLED("MAG: " + String(calibration[3]));
    digitalWrite(ERR_LED_PIN, HIGH);
    
    if (millis() - calStartTime > 5000) {  // 30 second timeout
        printOLED("MAG timeout!\nContinuing...");
        break;
    }
}
digitalWrite(ERR_LED_PIN, LOW);

  // Double-blink to signal ready
  digitalWrite(ERR_LED_PIN,    HIGH);
  digitalWrite(ServoStatusLED, HIGH);
  delay(500);
  digitalWrite(ERR_LED_PIN,    LOW);
  digitalWrite(ServoStatusLED, LOW);

  // Seed the PID timer so the first deltaTime isn't huge
  previousTime = millis();
}
 

void testWheel(){
    setMotorSpeed(255);   delay(250);
  setMotorSpeed(128);   delay(250);
  setMotorSpeed(0);     delay(250);
  setMotorSpeed(-128);  delay(250);
  setMotorSpeed(-255);  delay(250);
  setMotorSpeed(0);     delay(2000);

}
// ─────────────────────────────────────────────
//  calculateError(float current, float target)
//  Wraps the difference into -180 … +180 so the
//  controller always takes the shortest arc.
// ─────────────────────────────────────────────
float calculateError(float current, float target) {
  float difference = target - current;

  if (difference < -180) difference += 360;
  else if (difference > 180) difference -= 360;

  // Status LED: on when settled, off when correcting
  if (abs(difference) <= tolerance) {
    digitalWrite(ServoStatusLED, HIGH);
  } else {
    digitalWrite(ServoStatusLED, LOW);
  }

  return difference;
}


// ─────────────────────────────────────────────
//  fw_PID(float error)  →  float output
//  Standard PID with integral anti-windup.
// ─────────────────────────────────────────────
float PID(float error) {
  float currentTime = millis();
  float deltaTime   = (currentTime - previousTime) / 1000.0;

  // Proportional
  proportional = KP * error;

  // Integral with anti-windup clamp
  integral += KI * error * deltaTime;
  integral  = constrain(integral, -maxFlywheelSpeed, maxFlywheelSpeed);

  // Derivative (guard against divide-by-zero on first call)
  if (deltaTime > 0) {
    derivative = KD * (error - previousError) / deltaTime;
  }

  float output = round(proportional + integral + derivative);

  previousError = error;
  previousTime  = currentTime;

  return output;
}


// ─────────────────────────────────────────────
//  fw_mapPIDToMotor(float pidOutput)  →  int
//  Clamps PID output to the motor's PWM range.
// ─────────────────────────────────────────────
int fw_mapPIDToMotor(float pidOutput) {
  pidOutput = constrain(pidOutput, -maxFlywheelSpeed, maxFlywheelSpeed);
  return (int) round(pidOutput);
}


// ─────────────────────────────────────────────
//  FlywheelUpdate()
//  Call every loop iteration.
// ─────────────────────────────────────────────
void FlywheelUpdate() {
  float currentHeading = orientation[0];

  orientationError = calculateError(currentHeading, targetOrient);
  pidOutput        = PID(orientationError);
  motorCommand     = fw_mapPIDToMotor(pidOutput);

  // ── Deadband: ignore tiny commands the motor can't act on ──
  if (motorCommand > 0 && motorCommand < minFlywheelSpeed) {
    motorCommand = minFlywheelSpeed;
  } else if (motorCommand < 0 && motorCommand > -minFlywheelSpeed) {
    motorCommand = -minFlywheelSpeed;
  }

  // ── Saturation protection ─────────────────────────────────
  // Count how many consecutive updates we've been at full speed
  if (abs(motorCommand) >= maxFlywheelSpeed - 5) {
    saturationCounter++;
    if (saturationCounter >= SAT_COUNT_LIMIT && saturationTimer == 0) {
      saturationTimer = millis();  // Start the timer
    }
  } else if (abs(motorCommand) < maxFlywheelSpeed - 5
             && saturationCounter <= SAT_COUNT_LIMIT) {
    // Recovered before timeout — reset
    saturationCounter = 0;
    saturationTimer   = 0;
  }

  if ((millis() >= saturationTimer + SAT_TIME_LIMIT)
      && (saturationCounter >= SAT_COUNT_LIMIT)) {
    // Timed out and recovered — clear flags and resume
    saturationCounter = 0;
    saturationTimer   = 0;
  } else if (saturationCounter >= SAT_COUNT_LIMIT) {
    // Still saturated past limit — cut motor
    motorCommand = 0;
  }

  // Negative because spinning the flywheel CW pushes the box CCW
  setMotorSpeed(-motorCommand);
Serial.print("Target: "); Serial.print(targetOrient);
Serial.print("  Current: "); Serial.print(orientation[0]);
Serial.print("  Error: "); Serial.print(orientationError);
Serial.print("  CMD: "); Serial.println(motorCommand);

}
