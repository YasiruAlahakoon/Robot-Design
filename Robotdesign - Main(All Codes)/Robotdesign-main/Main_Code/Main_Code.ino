#include <VarSpeedServo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <TCA9548.h>
#include <NewPing.h>
#include <Encoder.h>
#include <VarSpeedServo.h>
#include <MPU6050.h>
#include <Wire.h>
#include <QTRSensors.h>
#include <arduinoFFT.h>

#define SAMPLES 128
#define SAMPLING_FREQUENCY 4000
#define TARGET_FREQUENCY_LOW 900
#define TARGET_FREQUENCY_HIGH 1100
#define ENCODER1_A 2
#define ENCODER1_B 3
#define ENCODER2_A 19
#define ENCODER2_B 18
#define leftmotorPWM 12
#define rightmotorPWM 13
#define motordir 8
#define triggerpinleft 38
#define echopinleft 40
#define triggerpinright 34
#define echopinright 36
#define maxdistance 200  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define sonarnum 2
#define S3 14
#define sensorOut 16
#define motorleftfront 53
#define motorleftback 51
#define motorrightfront 47
#define motorrightback 49

float duration;
const uint16_t avoiddistance = 150;
volatile int encoderPos = 0;
int lastEncoderPos = 0;
const int MPU = 0x68;  // MPU6050 I2C address
float GyroX, GyroY, GyroZ;
float gyroAngleZ;
float yaw;
float GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
const int tofcount = 6;
const int rightbackchannel = 3;  // Rear right channel of ToF
const int leftbackchannel = 4;   //Rear left channel of ToF
const int mindistance = 8;       //minimum distance for wall following US
const int numSensors = 9;
int irwhite = 520;
int ircolour = 520;
float KP = 2.78;
float KI = 0.001;
float KD = 17.5;
int KP_wall = 1;
int KD_wall = 0;
int sensorValues[numSensors];
int baseSpeed = 50;
int lastError = 0;
int integral = 0;
int tasknumber = 0;
bool checkpoint = false;
unsigned int pingSpeed = 100;
unsigned long pingTimer;
int previousleft;
int previousright;
bool notturned6 = true;
bool notturend7 = true;
int redMin = 12;    // Red minimum value
int redMax = 146;   // Red maximum value
int blueMin = 13;   // Blue minimum value
int blueMax = 161;  // Blue maximum value
int redPW = 0;
int bluePW = 0;
int redValue;
int blueValue;
int prevdirection = 0;
int blackthreshold;
int targetSpeed = 100;
int currentSpeed = 0;
int pwmValue = 100;
float last_time = 0;
float lastErrornew = 0;
float kp = 1.9;
float kd = 4;
float ki = 0;
float integralnew = 0;
const int sampleWindow = 100;  // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
float total_volts = 0;
float avg_volt;
int count;
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];
int frequency_detected = 0;
int left;
int right;
int encoderPosright = 0;
int encoderPosleft = 0;
bool notturned7 = true;
bool colour;

arduinoFFT FFT = arduinoFFT();
QTRSensors qtr;
TCA9548 multiplexer(0x70);
VL53L0X sensors[tofcount];
NewPing sonar[sonarnum] = {
  NewPing(triggerpinleft, echopinleft, maxdistance),
  NewPing(triggerpinright, echopinright, maxdistance),
};
MPU6050 mpu;
VarSpeedServo myservo1;
VarSpeedServo myservo2;
VarSpeedServo gripperServo;

void calibrate() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5, A6, A7, A8 }, numSensors);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 100; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
  digitalWrite(motorleftfront, HIGH);
  digitalWrite(motorrightfront, HIGH);
  tasknumber += 1;
  Serial.println(tasknumber);
}

void linefollow() {
  left = analogRead(A0);
  right = analogRead(A8);
  //Serial.println("---------");
  if (previousleft > irwhite || previousright > irwhite) {  // if previously black
    if (left < irwhite && right < irwhite) {                // if curently white
      delay(200);
      if (left < irwhite && right < irwhite) {  // if still white
        tasknumber += 1;
        digitalWrite(motorleftfront, HIGH);
        digitalWrite(motorrightfront, HIGH);
        analogWrite(leftmotorPWM, 80);
        analogWrite(rightmotorPWM, 80);
        if (analogRead(A0) < irwhite && analogRead(A8) > irwhite) {
          turnright();
        } else if (analogRead(A0) > irwhite && analogRead(A8) < irwhite) {
          turnleft();
        }
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        if (tasknumber == 3) {
          turnright();
        }
      }
    }
  }
  previousleft = left;
  previousright = right;
  int position = qtr.readLineWhite(sensorValues);
  // Serial.println(position);
  // Calculate PID control
  if (position == 0) {
    position = 4000;
  }
  if (position == 8000) {
    position = 4000;
  }
  int pidOutput = calculatePID(position);
  controlMotors(pidOutput);
}


int calculatePID(int position) {
  // Calculate the error
  int error = position - 4000;
  // Calculate the integral and derivative
  integral += error;
  int derivative = (error - lastError);

  // Calculate the output
  float output = KP * error + KI * integral + KD * derivative;

  // Update variables for the next iteration
  lastError = error;

  return int(output / 5);
}


void controlMotors(int pidOutput) {
  int leftSpeed = baseSpeed - pidOutput;
  int rightSpeed = baseSpeed + pidOutput;
  if (leftSpeed >= 0) {
    digitalWrite(motorleftback, LOW);
    digitalWrite(motorleftfront, HIGH);
  } else {
    digitalWrite(motorleftfront, LOW);
    digitalWrite(motorleftback, HIGH);
  }
  leftSpeed = constrain(leftSpeed, -90, 90);
  //pidmotorcontrol(leftSpeed, 0);
  analogWrite(leftmotorPWM, leftSpeed);
  if (rightSpeed >= 0) {
    digitalWrite(motorrightback, LOW);
    digitalWrite(motorrightfront, HIGH);
  } else {
    digitalWrite(motorrightfront, LOW);
    digitalWrite(motorrightback, HIGH);
  }
  rightSpeed = constrain(rightSpeed, -90, 90);
  analogWrite(rightmotorPWM, rightSpeed);
  //pidmotorcontrol(rightSpeed, 1);
}

void pidmotorcontrol(int target, bool side) {
  currentSpeed = calculateSpeed();
  float Error = target - currentSpeed;
  float derivative = (Error - lastErrornew);
  integralnew += Error;
  float pidError = kp * Error + kd * derivative + ki * integral;
  int PID_error = int(pidError);
  pwmValue += PID_error;
  Serial.print(pwmValue);
  pwmValue = constrain(pwmValue, -255, 255);
  lastError = Error;
  if (side) {
    analogWrite(rightmotorPWM, abs(pwmValue));
  } else {
    analogWrite(leftmotorPWM, abs(pwmValue));
  }
}

void isrAleft() {
  bool readA = digitalRead(ENCODER1_A);
  bool readB = digitalRead(ENCODER1_B);
  if (readB != readA) {
    encoderPosleft++;
  } else {
    encoderPosleft--;
  }
}
void isrBleft() {
  bool readA = digitalRead(ENCODER1_A);
  bool readB = digitalRead(ENCODER1_B);
  if (readA == readB) {
    encoderPosleft++;
  } else {
    encoderPosleft--;
  }
}
void isrAright() {
  bool readA = digitalRead(ENCODER2_A);
  bool readB = digitalRead(ENCODER2_B);
  if (readB != readA) {
    encoderPosright++;
  } else {
    encoderPosright--;
  }
}
void isrBright() {
  bool readA = digitalRead(ENCODER2_A);
  bool readB = digitalRead(ENCODER2_B);
  if (readA == readB) {
    encoderPosright++;
  } else {
    encoderPosright--;
  }
}
int calculateSpeed() {
  float current_time = millis();
  float elapsed_time = current_time - last_time;

  // Serial.print(" ENCODER POS: ");
  Serial.println(encoderPos - lastEncoderPos);
  Serial.println(elapsed_time);
  // Calculate speed based on encoder position changes
  float speed = (encoderPos - lastEncoderPos) * 60000 / (1632 * elapsed_time);  //speed in RPM//34*48=1632
  //float speed = (encoderPos - lastEncoderPos);
  //int speedInRPM = speed*(60/6.28);
  lastEncoderPos = encoderPos;
  last_time = current_time;

  return speed;
}

int calculatePIDWall(int error) {
  int derivative = error - lastError;
  int output = KP_wall * error + KD_wall * derivative;
  lastError = error;
  return int(output);
}

float angleofrotation() {
  previousTime = currentTime;                         // Previous time is stored before the actual time read
  currentTime = millis();                             // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);                    // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  GyroZ = GyroZ + GyroErrorZ;

  yaw = yaw + GyroZ * elapsedTime;
  return yaw;
}

void wallfollow(bool direction) {  // follow wall based on direction 0 for left and 1 for right
  if (direction) {
    //yaw = 0;
    turnleft();
    /*while (angleofrotation() < 45.0) {
    }*/
    //yaw = 0;
    digitalWrite(motorleftback, LOW);
    digitalWrite(motorleftfront, HIGH); /*
    multiplexer.selectChannel(2);  // Select the channel for the sensor
    uint16_t distance = sensors[2].readRangeContinuousMillimeters();
    while (distance > 300) {
      int error = error_calculate(direction);
      int pidOutput = calculatePIDWall(error);
      controlMotors(pidOutput);
    }
    while (distance < 300) {
      int error = error_calculate(direction);
      int pidOutput = calculatePIDWall(error);
      controlMotors(pidOutput);
    }*/
    linefollow();
  } else {
    turnright();
    digitalWrite(motorrightback, LOW);
    digitalWrite(motorrightfront, HIGH); /*
    multiplexer.selectChannel(5);  // Select the channel for the sensor
    uint16_t distance = sensors[5].readRangeContinuousMillimeters();
    while (distance > 300) {
      int error = error_calculate(direction);
      int pidOutput = calculatePIDWall(error);
      controlMotors(pidOutput);
    }
    while (distance < 300) {
      int error = error_calculate(direction);
      int pidOutput = calculatePIDWall(error);
      controlMotors(pidOutput);
    }*/
    linefollow();
  }
}

float echoCheck(int i) {
  float ultrasonicdistance;
  if (sonar[i].check_timer()) {
    ultrasonicdistance = sonar[i].ping_result / US_ROUNDTRIP_CM;
  }
  return (ultrasonicdistance);
}

int read_tof(uint8_t i) {
  multiplexer.selectChannel(i);  // Select the channel for the sensor
  uint16_t distance = sensors[i].readRangeContinuousMillimeters();
  if (sensors[i].timeoutOccurred()) {
    distance = 700;
  }
  return (distance);
}


int error_calculate(bool direction) {
  int distance_array[2];
  int error;
  if (direction == 0) {
    for (uint8_t i = 0; i < 2; i++) {
      distance_array[i] = read_tof(i);
    }
    error = distance_array[0] - distance_array[1];
  } else {
    for (uint8_t i = 3; i < 5; i++) {
      distance_array[i] = read_tof(i);
    }
    error = distance_array[1] - distance_array[0];
  }
  return (error);
}

/*
void grabbox() {
  digitalWrite(motorleftfront, LOW);
  digitalWrite(motorleftback, LOW);
  digitalWrite(motorrightfront, LOW);
  digitalWrite(motorrightback, LOW);
  armgrab();
  mpu.getMotion6();
  float gyroX = mpu.getRotationX();
  float rotation angle = 0;
  digitalWrite(motorleftfront, HIGH);
  digitalWrite(motorrightback, HIGH);
  while (rotationangle < 180.0) {
    rotationSpeed += gyroX / 131.0;  // MPU6050 sensitivity: 131 LSB/°/s
    rotationAngle += rotationSpeed * 0.01;
    analogWrite(leftmotorPWM, 80);
    analogWrite(rightmotorPWM, 80);
  }
}
*/
void turnright() {
  digitalWrite(motorrightfront, LOW);
  digitalWrite(motorrightback, HIGH);
  digitalWrite(motorleftfront, HIGH);
  digitalWrite(motorleftback, LOW);
  analogWrite(leftmotorPWM, 120);
  analogWrite(rightmotorPWM, 120);
  delay(500);
  digitalWrite(motorleftfront, LOW);
  digitalWrite(motorrightback, LOW);
}

void turnleft() {
  digitalWrite(motorleftfront, LOW);
  digitalWrite(motorleftback, HIGH);
  digitalWrite(motorrightfront, HIGH);
  digitalWrite(motorrightback, LOW);
  analogWrite(leftmotorPWM, 120);
  analogWrite(rightmotorPWM, 120);
  delay(500);
  digitalWrite(motorrightfront, LOW);
  digitalWrite(motorleftback, LOW);
}

void aboutturn() {
  digitalWrite(motorleftfront, LOW);
  digitalWrite(motorleftback, HIGH);
  analogWrite(leftmotorPWM, 80);
  analogWrite(rightmotorPWM, 80);
  while (angleofrotation() < 180.0) {
  }
  yaw = 0;
  digitalWrite(motorleftback, LOW);
  digitalWrite(motorleftfront, HIGH);
}

bool checkcolour() {
  int red = getRedPW();
  int blue = getBluePW();
  if (red > blue) {
    return true;
  } else {
    return false;
  }
}

int getRedPW() {
  digitalWrite(S3, LOW);
  int PW;
  PW = pulseIn(sensorOut, LOW);
  return PW;
}

int getBluePW() {
  digitalWrite(S3, LOW);
  int PW;
  PW = pulseIn(sensorOut, LOW);
  return PW;
}

void linefollowwithjunctions() { /*
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
  if (sensorValues[0] > ircolour && sensorValues[numSensors - 1] > ircolour) {  // go right at a t junction
    turnright();
    previousleft = sensorValues[0];
    previousright = sensorValues[5];
    int pidOutput = calculatePID(sensorValues);
    controlMotors(pidOutput);
  } else if (sensorValues[numSensors - 1] > 0) {  // if only right sensor reports line, turn right
    turnright();
    previousleft = sensorValues[0];
    previousright = sensorValues[5];
    int pidOutput = calculatePID(sensorValues);
    controlMotors(pidOutput);
  } else if (sensorValues[0] > ircolour) {  // if only left sensor reports a line, go straight
    controlMotors(0);
  } else if (sensorValues[0] < blackthreshold && sensorValues[1] < blackthreshold && sensorValues[4] < blackthreshold && sensorValues[5] < blackthreshold) {  // if all are black
    delay(1000);
    aboutturn();
  }*/
}

void calculate_IMU_error() {
  c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroZ = Wire.read() << 8 | Wire.read();
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  GyroErrorZ = GyroErrorZ / 200;
}


void linefollowuntilright() {
}

void guardrobot() {
  int leftdistance = echoCheck(0);
  int rightdistance = echoCheck(1);
  if (rightdistance < 50) {
    while (leftdistance > 50) {
    }
    linefollowwithjunctions();
  }
}

void gradualGrip() {
  for (int pos = 100; pos <= 135; pos += 1) {
    gripperServo.write(pos);
    delay(15);
  }
}

bool Sense_colourbox(int position1, int position2, int speed1, int speed2) {
  const int PWM_1_DEGREE = 500;
  const int PWM_180_DEGREES = 2500;
  float targetMicros1 = map(position1, 0, 180, PWM_1_DEGREE, PWM_180_DEGREES);
  float targetMicros2 = map(position2, 0, 180, PWM_1_DEGREE, PWM_180_DEGREES);
  myservo1.write(targetMicros1, speed1);
  myservo1.wait();
  myservo2.write(targetMicros2, speed2);
  myservo2.wait();
  bool boxcolour2 = checkcolour();
  return boxcolour2;
}

void movegripper(int position1, int position2, int speed1, int speed2) {
  const int PWM_1_DEGREE = 500;
  const int PWM_180_DEGREES = 2500;
  float targetMicros1 = map(position1, 0, 180, PWM_1_DEGREE, PWM_180_DEGREES);
  float targetMicros2 = map(position2, 0, 180, PWM_1_DEGREE, PWM_180_DEGREES);
  myservo1.write(targetMicros1, speed1);
  myservo1.wait();
  myservo2.write(targetMicros2, speed2);
  myservo2.wait();
}

void mic_calibration() {
  unsigned long startMillis = millis();  // Start of sample window
  unsigned int peakToPeak = 0;           // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  // collect data for 50
  for (int i = 1; i < 2000; i++) {
    while (millis() - startMillis < sampleWindow) {
      sample = analogRead(0);
      if (sample < 1024)  // toss out spurious readings
      {
        if (sample > signalMax) {
          signalMax = sample;  // save just the max levels
        } else if (sample < signalMin) {
          signalMin = sample;  // save just the min levels
        }
      }
    }
    peakToPeak = signalMax - signalMin;        // max - min = peak-peak amplitude
    double volts = (peakToPeak * 5.0) / 1024;  // convert to volts
    count += 1;
    total_volts += volts;
  }

  avg_volt = total_volts / 2000;
  //Serial.println(count);
  //Serial.println(avg_volt);
}
bool mic() {
  unsigned long startMillis = millis();  // Start of sample window
  unsigned int peakToPeak = 0;           // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;


  while (millis() - startMillis < sampleWindow) {
    sample = analogRead(A9);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax) {
        signalMax = sample;  // save just the max levels
      } else if (sample < signalMin) {
        signalMin = sample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;        // max - min = peak-peak amplitude
  double volts = (peakToPeak * 5.0) / 1024;  // convert to volts
  frequency_detection();
  if (frequency_detected == 1) {
    return true;
  } else {
    return false;
  }
}

bool frequency_detection() {
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long timing = micros();
    vReal[i] = analogRead(A0);
    vImag[i] = 0;


    while (micros() < (microseconds + sampling_period_us)) {
    }
  }

  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  double dominant_frequency = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  if (dominant_frequency >= 375 && dominant_frequency <= 625) {
    frequency_detected = 1;

  } else {
    frequency_detected = 0;
  }
  return (frequency_detected);
}

void setup() {
  Wire.begin();
  mpu.initialize();
  multiplexer.begin();
  pingTimer = millis();
  for (uint8_t i = 0; i < tofcount; ++i) {
    multiplexer.selectChannel(i);  // Select the channel
    sensors[i].init();
    sensors[i].setTimeout(500);
    sensors[i].setMeasurementTimingBudget(20000);  // Set measurement timing budget (adjust as needed)
    sensors[i].startContinuous();
  }
  pinMode(45, INPUT_PULLUP);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  myservo1.attach(9);
  myservo2.attach(10);
  gripperServo.attach(6);
  gripperServo.write(180);
  myservo1.write(150);
  myservo2.write(180);
  pinMode(leftmotorPWM, OUTPUT);
  pinMode(leftmotorPWM, OUTPUT);
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);
  digitalWrite(48, HIGH);
  digitalWrite(50, HIGH);
  digitalWrite(motorleftfront, LOW);
  digitalWrite(motorleftback, LOW);
  digitalWrite(motorrightfront, LOW);
  digitalWrite(motorrightback, LOW);
  pinMode(leftmotorPWM, OUTPUT);
  pinMode(rightmotorPWM, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), isrAleft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_B), isrBleft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), isrAright, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_B), isrBright, CHANGE);
  mic_calibration();
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  Serial.begin(9600);
}

void loop() {
  if (digitalRead(45) == LOW) {
    tasknumber = 3;
  }
  if (tasknumber == 0) {
    calibrate();
  } else if (tasknumber == 1) {
    linefollow();
  } else if (tasknumber == 2) {
    linefollow();
    digitalWrite(triggerpinleft, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerpinleft, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerpinleft, LOW);
    duration = pulseIn(echopinleft, HIGH);
    float leftreading = duration * 0.034 / 2;
    digitalWrite(triggerpinright, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerpinright, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerpinright, LOW);
    duration = pulseIn(echopinleft, HIGH);
    float rightreading = duration * 0.034 / 2;
    if (leftreading < mindistance) {
      wallfollow(0);
    } else if (rightreading < mindistance) {
      wallfollow(1);
    }
  } else if (tasknumber == 3) {
    baseSpeed = 140;
    KP = 2.78;
    KI = 0;
    KD = 15.5;
    linefollow();
  } else if (tasknumber == 4) {
    linefollow();
    multiplexer.selectChannel(6);
    uint16_t distance = sensors[6].readRangeContinuousMillimeters();
    if (distance < avoiddistance) {
      analogWrite(leftmotorPWM, 50);
      analogWrite(rightmotorPWM, 50);
      // task number is also incremented here
    }
    if (distance < 50) {
      digitalWrite(motorleftfront, LOW);
      digitalWrite(motorrightfront, LOW);
      gradualGrip();
      digitalWrite(motorleftback, HIGH);
      digitalWrite(motorrightback, HIGH);
    }
  } else if (tasknumber == 5) {
    linefollow();
    if (analogRead(A5) < irwhite && analogRead(A4) < irwhite && analogRead(A3) < irwhite && analogRead(A0) > irwhite) {  // if left sensors detect a white line
      turnleft();
    }
  } else if (tasknumber == 6) {
    if (notturned6) {
      turnleft();
      notturned6 = false;
    }
    linefollow();
  } else if (tasknumber == 7) {
    if (notturned7) {
      turnright();
      notturned7 = false;
    }
    linefollow();
  } else if (tasknumber == 8) {
    multiplexer.selectChannel(6);
    uint16_t distance1 = sensors[6].readRangeContinuousMillimeters();
    if (distance1 < 50) {
      digitalWrite(motorleftfront, LOW);
      digitalWrite(motorrightfront, LOW);
      bool colour = Sense_colourbox(85, 95, 100, 100);
      movegripper(110, 70, 100, 100);
      digitalWrite(motorleftback, HIGH);
      digitalWrite(motorrightback, HIGH);
      analogWrite(leftmotorPWM, 40);
      analogWrite(rightmotorPWM, 40);
      while (analogRead(A0) > irwhite && analogRead(A1) > irwhite && analogRead(A4) > irwhite && analogRead(A5) > irwhite) {
      }
    }
    if (analogRead(A0) > irwhite && analogRead(A1) > irwhite && analogRead(A4) > irwhite && analogRead(A5) > irwhite) {
      digitalWrite(motorleftback, LOW);
      digitalWrite(motorrightback, LOW);
      digitalWrite(motorrightback, HIGH);
      digitalWrite(motorleftback, HIGH);
      tasknumber += 1;
    }
  } else if (tasknumber == 9) {
    turnright();
    delay(3000);
    linefollow();
    bool linecolour = checkcolour();
    if (linecolour == colour) {
      tasknumber += 1;
    } else {
      int distance = 0;
      aboutturn();
      while (distance < 70) {
        linefollow();
      }
      tasknumber += 1;
    }
  } else if (tasknumber == 10) {
    linefollowwithjunctions();
  } else if (tasknumber == 11) {
    if (!mic()) {
      digitalWrite(motorrightfront, HIGH);
      digitalWrite(motorleftfront, HIGH);
      linefollow();
    } else {
      digitalWrite(motorleftfront, LOW);
      digitalWrite(motorleftback, LOW);
      digitalWrite(motorrightfront, LOW);
      digitalWrite(motorrightback, LOW);
    }
  } else if (tasknumber == 12) {
    guardrobot();
  }
}