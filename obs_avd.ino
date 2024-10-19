#include <Servo.h>

#define PIN_SERVO           2       
#define MOTOR_DIRECTION     0
#define PIN_DIRECTION_LEFT  4
#define PIN_DIRECTION_RIGHT 3
#define PIN_MOTOR_PWM_LEFT  6
#define PIN_MOTOR_PWM_RIGHT 5
#define PIN_SONIC_TRIG      7
#define PIN_SONIC_ECHO      8
#define PIN_BATTERY         A0

#define OBSTACLE_DISTANCE   30   // 10cm threshold for detecting obstacles
#define MAX_DISTANCE        1000 
#define SONIC_TIMEOUT       (MAX_DISTANCE * 60)
#define SOUND_VELOCITY      340

Servo servo;
byte servoOffset = 0;
int speedOffset;  // batteryVoltageCompensationToSpeed

// Variables for control loop
int furthestAngle = 90;
int furthestDistance = 0;
int dTtravel = 0;
int dTturn = 0;

void setup() {
  pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);
  pinMode(PIN_SONIC_TRIG, OUTPUT);
  pinMode(PIN_SONIC_ECHO, INPUT);
  servo.attach(PIN_SERVO);
  calculateVoltageCompensation();
}

void loop() {
  updateControlLoop();
}

void updateControlLoop() {
  // Step 1: Move forward until obstacle detected within 10cm
  while (getSonarAverage() > OBSTACLE_DISTANCE) {
    motorRun(100 + speedOffset, 100 + speedOffset);  // Move forward
  }
  motorRun(0, 0);  // Stop when object is detected

  // Step 2: Sweep with more frequent angle checks (every 30 degrees)
  int numAngles = 13;
  int distances[numAngles];
  int angles[numAngles];
  int stepAngle = 30;  // Sweep step size (in degrees)
  int baseAngle = 15;  // Start angle

  // Populate angles from 30 to 150 degrees (frequent checks)
  for (int i = 0; i < numAngles; i++) {
    angles[i] = baseAngle + (i * stepAngle);  // 30, 40, 50, ..., 150
    servo.write(angles[i]);
    delay(130);  // Wait for servo to reach position
    distances[i] = getSonarAverage();  // Get averaged sonar reading
  }

  // Find the furthest distance and corresponding angle
  furthestDistance = distances[0];
  furthestAngle = angles[0];
  for (int i = 1; i < numAngles; i++) {
    if (distances[i] > furthestDistance) {
      furthestDistance = distances[i];
      furthestAngle = angles[i];
    }
  }

  // Step 3: Rotate sensor back to center
  servo.write(90);
  delay(200);

  // Step 4: Dynamically turn robot until the distance matches the furthest distance
  unsigned long startTurn = millis();  // Track turning duration
  if (furthestAngle > 90) {
    // Turn right to match the recorded distance

    while (true){
      int a = getSonarAverage();
      if (a > (furthestDistance - 0) || a >= MAX_DISTANCE){
        break;
      }
      motorRun(-(60 + speedOffset), 60 + speedOffset);  // Turn right
      if (millis() - startTurn > 5000) break;  // Timeout to avoid getting stuck
    }
  } else if (furthestAngle < 90) {
    // Turn left to match the recorded distance
    while(true){
      int a = getSonarAverage();
      if (a > (furthestDistance - 20) || a >= MAX_DISTANCE){
        break;
      }
      motorRun(60+ speedOffset, -(60+ speedOffset));  // Turn left
      if (millis() - startTurn > 5000) break;  // Timeout to avoid getting stuck
    }
  }
  motorRun(0, 0);  // Stop turning when distance is close to the recorded one

  dTturn = millis() - startTurn;  // Save turn duration

  // Step 5: Move forward and turn sensor to the opposite side of the turn to detect passing object
  motorRun(100 + speedOffset, 100 + speedOffset);
  int lastDistance = getSonarAverage();
  dTtravel = 0;

  // **Turn the sensor to the opposite side of the turn**
  if (furthestAngle > 90) {
    // If the robot turned right, turn the sensor to the left (30 degrees)
    servo.write(15);
  } else if (furthestAngle < 90) {
    // If the robot turned left, turn the sensor to the right (150 degrees)
    servo.write(165);
  }
  delay(200);  // Let the sensor settle

  // Now, move forward while checking for distance increase and decrease
  unsigned long startTime = millis();  // Track time spent moving forward
  while (true) {
    int currentDistance = getSonarAverage();
    if (currentDistance > lastDistance && currentDistance < furthestDistance) {
      dTtravel += 100;  // Increment travel time
    } else if (currentDistance < lastDistance) {
      break;  // Object passed
    }
    
    // Timeout to avoid getting stuck
    if (millis() - startTime > 5000) {  // 5 second timeout
      break;  // Exit if timeout reached
    }

    lastDistance = currentDistance;
    delay(100);  // Travel loop delay
  }
  motorRun(0, 0);  // Stop after object is passed
  servo.write(90);
  // Step 6: Turn back the amount of dTturn
  motorRun(-(100+ speedOffset), 100+ speedOffset);  // Reverse turn
  delay(dTturn);  // Duration for turning back
  motorRun(0, 0);

  // Step 7: Move forward the amount of dTtravel
  motorRun(100 + speedOffset, 100 + speedOffset);
  delay(dTtravel);
  motorRun(0, 0);

  // Step 8: Loop the entire thing
}

// Function to get averaged sonar reading
float getSonarAverage() {
  float sumDistance = 0;
  int numReadings = 5;  // Take 5 readings for averaging
  for (int i = 0; i < numReadings; i++) {
    sumDistance += getSonar();
    delayMicroseconds(2 * SONIC_TIMEOUT);  // Delay between readings
  }
  return sumDistance / numReadings;  // Return average distance
}

// Function to get a single sonar reading
float getSonar() {
  unsigned long pingTime;
  float distance;
  digitalWrite(PIN_SONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONIC_TRIG, LOW);
  pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT);
  if (pingTime != 0)
    distance = (float)pingTime * SOUND_VELOCITY / 2 / 10000;
  else
    distance = MAX_DISTANCE;
  return distance;
}

void calculateVoltageCompensation() {
  float voltageOffset = 8.4 - getBatteryVoltage();
  speedOffset = voltageOffset * 20;
}

void motorRun(int speedl, int speedr) {
  int dirL = 0, dirR = 0;
  if (speedl < 0) {
    dirL = 0 ^ MOTOR_DIRECTION;
  } else {
    dirL = 1 ^ MOTOR_DIRECTION;
    speedl = -speedl;
  }

  if (speedr < 0) {
    dirR = 1 ^ MOTOR_DIRECTION;
  } else {
    dirR = 0 ^ MOTOR_DIRECTION;
    speedr = -speedr;
  }

  digitalWrite(PIN_DIRECTION_LEFT, dirL);
  digitalWrite(PIN_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_MOTOR_PWM_LEFT, speedl);
  analogWrite(PIN_MOTOR_PWM_RIGHT, speedr);
}

float getBatteryVoltage() {
  pinMode(PIN_BATTERY, INPUT);
  int batteryADC = analogRead(PIN_BATTERY);
  float batteryVoltage = batteryADC / 1023.0 * 5.0 * 4;
  return batteryVoltage;
}
