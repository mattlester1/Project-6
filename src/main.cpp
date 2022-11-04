#include <Arduino.h>
#include <Encoder.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>


// defining motor and encoder pins
#define Motor_Pin1 2
#define Motor_Pin2 3
Encoder encode(30, 31);


float PPR = 405;


Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
float pitch, roll;
volatile float pitchOffset = 0.0f;
volatile float rollOffset = 0.0f;
float prScalar = 7.8;

int incomingByte = 0;

float eror_integ = 0.0;

volatile float ref_theta;
volatile float motor_theta;
volatile float thetaError = 0;
volatile float oldError = 0;

float k_p = 5;
volatile float p_control = 0;
float k_i = .01;
volatile float i_control = 0;
int oldTime;
int newTime;
int deltaT;
int power;

void processIMU();
float processIMUAngle();
float readMotorAngle();
float PI_Control();
void sendPWM();
void processIMU();
void Compensate();
int PowerControl();

void setup()
{
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
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
  // ref_theta = processIMUAngle(); 
}

void loop()
{
  // put your main code here, to run repeatedly:
  // Serial.println(processIMUAngle());
  // Serial.println(readMotorAngle());
  // Serial.print("\t");
  // Serial.println(readAngle());
  Compensate();
  // processIMU();
}

void processIMU()
{
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

  // Serial.println(a.acceleration.roll);

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}

float processIMUAngle()
{

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float angleOffSet = -2.0;

  return ((180 / M_PI) * atan2(a.acceleration.x, a.acceleration.z)) + angleOffSet;
  
}

float readMotorAngle()
{
  return (encode.read() / PPR) * 360;
}


float PI_Control(){
  newTime = millis();
  // ref_theta = processIMUAngle();
  motor_theta = readMotorAngle();

  thetaError = ref_theta - motor_theta;
  p_control = k_p * thetaError; // Seems to be working, creates occelations, may need refinment

  oldTime = newTime;
  oldError = thetaError;

  newTime = millis();
  // ref_theta = processIMUAngle();
  motor_theta = readMotorAngle();

  deltaT = (newTime - oldTime) * 1000;
  thetaError = ref_theta - motor_theta;
  i_control = k_i * (i_control + ((thetaError - oldError) * deltaT));
  return p_control + i_control;


}


void Compensate()
{

  float comp = -1 * PI_Control();



    if (comp > 0){

      analogWrite(Motor_Pin1, 0);
      analogWrite(Motor_Pin2, PowerControl());
     }

    else{
      analogWrite(Motor_Pin1, PowerControl());
      analogWrite(Motor_Pin2, 0);

    }
  }

  int PowerControl(){

    if (abs(PI_Control()) < 10){
      power = 100;
    }
    else if (abs(PI_Control()) < 20){
      power = 150;
    }
    else if (abs(PI_Control()) < 30){
      power = 200;
    }
    else if (abs(PI_Control()) < 40){
      power = 225;
    }
    else{
      power = 255;
    }
    return power;
  }

  // while ((abs(IMUAngle - motorAngle)) > 0.5)
  // {




  //   // if (refAngle < 0)
  //   // {
  //   //   analogWrite(Motor_Pin1, 255);
  //   //   analogWrite(Motor_Pin2, 0);
  //   //   // motorAngle = encoderAngle;
  //   // }
  //   // else
  //   // {
  //   //   analogWrite(Motor_Pin1, 0);
  //   //   analogWrite(Motor_Pin2, 255);
  //   //   // motorAngle = encoderAngle;
  //   // }

  //   // analogWrite(Motor_Pin1, 0);
  //   // analogWrite(Motor_Pin2, 0);
  //   // encoderAngle;
  // }
// }
