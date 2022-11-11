#include <Arduino.h>
#include <Encoder.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

#define Motor_Pin1 2
#define Motor_Pin2 3
Encoder encode(30, 31);

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

const float PPR = 405;
const int zero_speed = 0;
const float k_p = 1;
const float k_i = 1.35;
volatile float ref_theta;
volatile float motor_theta;
volatile float thetaError = 0;
volatile float oldError = 0;
volatile float p_control = 0;
volatile float i_control = 0;
volatile int oldTime;
volatile int newTime;
int deltaT = 0;
int power = 0;

void processIMU();
void Compensate(float AngleDifference, float Speed);
float processIMUAngle();
float readMotorAngle();
float PI_Control(float IMUAngle, float MotorAngle);
int PowerControl();

void setup(){
  // put your setup code here, to run once:
  Serial.begin(115200);

  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (true)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 found!");

  // set accelerometer range to +-2G
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  delay(100);
}

void loop(){
  Compensate(PI_Control(processIMUAngle(), readMotorAngle()), PowerControl());
  // processIMU();

}

void processIMU(){
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}

// reads IMU info and returns angle of breadboard about the Y-axis
float processIMUAngle(){

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float angleOffSet = 0;

  return ((180 / M_PI) * atan2(a.acceleration.x, a.acceleration.z)) + angleOffSet;
}

// reads motor encoder to determine the angle of motor shaft
float readMotorAngle(){

  return (encode.read() / PPR) * 360;
}


// Includes proportional and Integral control
float PI_Control(float IMUAngle, float MotorAngle){

  newTime = millis();
  ref_theta = IMUAngle;
  motor_theta = MotorAngle;

  thetaError = ref_theta + motor_theta;
  p_control = k_p * thetaError; 

  oldTime = newTime;
  oldError = thetaError;

  newTime = millis();
  ref_theta = IMUAngle;
  motor_theta = MotorAngle;

  deltaT = (newTime - oldTime) * (1/1000); // time difference and converting from milliseconds to seconds
  thetaError = ref_theta + motor_theta;
  i_control = k_i * (i_control + ((thetaError - oldError) * deltaT));

  return p_control + i_control;
}


// Takes results from PI_Control() and PowerControl() and applies them to motor
void Compensate(float AngleDifference, float Speed){

  float comp = AngleDifference;

    if (comp == 0){
      analogWrite(Motor_Pin1, zero_speed);
      analogWrite(Motor_Pin2, zero_speed);
    }
    else if (comp > 0){

      analogWrite(Motor_Pin1, Speed);
      analogWrite(Motor_Pin2, zero_speed);
     }

    else{
      analogWrite(Motor_Pin1, zero_speed);
      analogWrite(Motor_Pin2, Speed);

    }
  }


  // controls power output depending on angle difference. 
  //The greater the difference the the greater the power
  int PowerControl(){

    float angleError = PI_Control(processIMUAngle(), readMotorAngle());

    if (abs(angleError) < 5){
      power = 150;
    }
    else if (abs(angleError) < 10){
      power = 175;
    }
    else if (abs(angleError) < 25){
      power = 200;
    }
    else if (abs(angleError) < 40){
      power = 225;
    }
    else{
      power = 255;
    }

    return power;
  }
