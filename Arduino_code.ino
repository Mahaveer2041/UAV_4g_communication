#include <Wire.h>
#include <MPU6050_light.h>
MPU6050 mpu(Wire);
#include <Servo.h>
#include <IBusBM.h>

#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 125  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 500  // this is the 'maximum' pulse length count (out of 4096)
String data = "";
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
int AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };
int previousAngleRoll=0;
int previousAnglePitch=0;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
  AnglePitch = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AngleRoll = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
//   AngleRoll = roundAngle(AngleRoll);
//   AnglePitch = roundAngle(AnglePitch);
}
int previousAngle=0;

int roundAngle(int angle) {
  int differenceRoll = abs(angle - previousAngleRoll);
  int differencePitch = abs(angle - previousAnglePitch);
  
  if (differenceRoll >= 6 || differencePitch >= 6) {
    int roundedAngle = round(angle / 6.0) * 6;
    if (differenceRoll >= 6) {
      previousAngleRoll = roundedAngle;
    }
    if (differencePitch >= 6) {
      previousAnglePitch = roundedAngle;
    }
    return roundedAngle;
  } else {
    previousAngleRoll = angle;
    previousAnglePitch = angle;
    return angle;
  }
}

IBusBM ibus;
float gyroX = 0.0, gyroY = 0.0;
int RXvalues[6];
int ch3_val=1000;
int ch1_val=1000;
int ch2_val=1000;
int ch4_val=1000;
int ch6_val=1000;

float desiredRollAngle = 0.0;
float desiredElevatorAngle = 0.0;

int pidRollInput, pidPitchInput;
int pidRollOutput, pidPitchOutput;
float pidRollKp = 3, pidRollKi = 0, pidRollKd = 0;
float pidPitchKp = 3, pidPitchKi = 0, pidPitchKd =0;
float pidRollLastError = 0.0, pidPitchLastError = 0.0;
float pidRollIntegral = 0.0, pidPitchIntegral = 0.0;

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}



void setup() {
  pwm.begin();
  
  pwm.setPWMFreq(50);
  Wire.setClock(400000);
  Wire.begin();
  Serial.begin(115200);
  Serial.setTimeout(20);
  //Serial.setTimeout(0.05);
  // delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  LoopTimer = micros();
  Serial.println("setup Done!\n");
  ibus.begin(Serial3);
}

void loop() {
  // Serial.println("Done!\n");

  // gyro_signals();
  // Serial.print("Roll Angle [°] ");
  // Serial.print(AngleRoll);
  // Serial.print(" Pitch Angle [°] ");
  // Serial.println(AnglePitch);

  readRX();
  int ch5_val = RXvalues[4];
  if(Serial3.available()>0 && ch5_val>1500)
              {
              readRX();
              ch3_val = RXvalues[2];
              // Serial.println(ch3_val);
              ch1_val = RXvalues[0];
              ch2_val = RXvalues[1];
              ch4_val = RXvalues[3];
              ch5_val = RXvalues[4];
              ch6_val = RXvalues[5];


                    if (ch6_val > 1500) {
                                pwm.setPWM(4, 0, pulseToServo(ch3_val));
                                if ((ch1_val < 1800 && ch1_val > 1300)) {
                                  gyro_signals();
                                  pidRollInput =  map(AngleRoll,-90,90,1000,2000);
                                  pidPitchOutput = calculatePID(pidPitchInput, ch2_val, pidPitchLastError, pidPitchIntegral, pidPitchKp, pidPitchKi, pidPitchKd);
                                  pwm.setPWM(2, 0, pulseToServo(ch1_val + pidRollOutput));
                                }
                                if ((ch2_val < 1800 && ch2_val > 1300)) {
                                  gyro_signals();
                                  pidPitchInput =  map(AnglePitch,-90,90,1000,2000);
                                  pidRollOutput = calculatePID(pidRollInput, ch1_val, pidRollLastError, pidRollIntegral, pidRollKp, pidRollKi, pidRollKd);
                                  pwm.setPWM(0, 0, pulseToServo(ch2_val + pidPitchOutput));
                                  pwm.setPWM(1, 0, pulseToServo(ch2_val - pidPitchOutput));
                                }
                     }else {
                      pwm.setPWM(0, 0, pulseToServo(ch2_val));
                      pwm.setPWM(1, 0, pulseToServo(3000-ch2_val));
                      pwm.setPWM(2, 0, pulseToServo(ch1_val));
                      pwm.setPWM(4, 0, pulseToServo(ch3_val));
                    
                    }
              }
  
 if (Serial.available() > 0) {
    // Read the incoming data until a newline character is encountered
    String data = Serial.readStringUntil('\n');
    
    // Split the data by commas
    int values[5]; // Assuming 5 channels
    int index = 0;
    char *ptr = strtok((char *)data.c_str(), ",");
    while (ptr != NULL && index < 5) {
        values[index++] = atoi(ptr);
        ptr = strtok(NULL, ",");
    }

    // Set servo positions manually based on received channel values
    // Manually set PWM values for each servo
    pwm.setPWM(0, 0, pulseToServo(values[0])); 
    pwm.setPWM(1, 0, pulseToServo(3000-values[0])); 
    Serial.println(values[0]);
   

  // Channel 1
    pwm.setPWM(2, 0, pulseToServo(values[1]));
     Serial.println(values[0]);
    
    // Channel 2
    pwm.setPWM(4, 0, pulseToServo(values[2]));
     Serial.println(values[0]);
   
    // Serial.println(values[2]); // Channel 3
    // pwm.setPWM(3, 0, pulseToServo(values[3])); // Channel 4
    // pwm.setPWM(4, 0, pulseToServo(values[4])); // Channel 5
}
  
}

void readRX() {
  for (byte i = 0; i < 6; i++) {
    RXvalues[i] = readChannel(i, 1000, 2000, 0);
  }
}

int calculatePID(float input, float setpoint, float &lastError, float &integral, float kp, float ki, float kd) {
  float error = setpoint - input;
  integral += error;
  float derivative = error - lastError;
  float output = kp * error + ki * integral + kd * derivative;
  lastError = error;
  return constrain(output, -300, 300);
}

int pulseToServo(int ang) {
  int roundedAngle = round(ang);
  int pulse = map(roundedAngle, 1000, 2000, SERVOMIN, SERVOMAX);
  return pulse;
}
