#include <Servo.h>
#include <MPU6050.h>
#include <Wire.h>

// Defining the motors
Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

#define CHANNEL1 1 
#define CHANNEL2 2
#define CHANNEL3 3 
#define CHANNEL4 4
#define CHANNEL5 5 
#define CHANNEL6 6

#define channel_pin1  6
#define channel_pin2  7
#define channel_pin3  8
#define channel_pin4  9
#define channel_pin5  10
#define channel_pin6  11

#define PITCH 0
#define ROLL 1
#define YAW 2

float currTime = 0;
float prevTime = 0;
float timeError = 0;

double Kp[3] = {1, 1, 1};
double Ki[3] = {1, 1, 1};
double Kd[3] = {1, 1, 1};


int channel[6] = {0, 0, 0, 0, 0, 0};    // receiver data in the order ,Throttle, rot1, rot2

double errors[3] = {0, 0, 0};     // errors in the order: PITCH ROLL YAW
double prevErrors[3] = {0, 0, 0};   // Previous errors in the order: PITCH ROLL YAW
double currAngles[3] = {0, 0, 0};    // curr angles errors in the order: PITCH ROLL YAW

MPU6050 mpu;

void setup() {

  Serial.begin(9600);   
  Serial.println("Initializing MPU6050");
  Wire.begin();
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);

  Motor1.attach(5, 1000, 2000);
  Motor2.attach(6, 1000, 2000);
  Motor3.attach(9, 1000, 2000);
  Motor4.attach(10, 1000, 2000);
  
   
}

void loop() {
  currTime = millis();
  timeError = currTime - prevTime;

  // getReceiverData();

  // double desiredRateThrottle = channel[CHANNEL1];
  // double desiredRatePitch    = 0.15 * (channel[CHANNEL3] - 1500);
  // double desiredRateRoll     = 0.15 * (channel[CHANNEL4] - 1500);
  // double desiredRateYaw      = 0.15 * (channel[CHANNEL2] - 1500);

  getMPUData();

  for(int i=0;i<3;i++) {
    Serial.print(currAngles[i]);
    Serial.print(' ');
    delay(100);
  }
  Serial.println("");

  // errors[0] = desiredRatePitch - currAngles[0];
  // errors[1] = desiredRateRoll - currAngles[1];
  // errors[2] = desiredRateYaw - currAngles[2];

  // double roll   = computePID(errors[ROLL], prevErrors[ROLL], Kp[ROLL], Ki[ROLL], Kd[ROLL], timeError);
  // double yaw    = computePID(errors[YAW], prevErrors[YAW], Kp[YAW], Ki[YAW], Kd[YAW], timeError);
  // double pitch  = computePID(errors[PITCH], prevErrors[PITCH], Kp[PITCH], Ki[PITCH], Kd[PITCH], timeError);

  // for(int i=0;i<3;i++) prevErrors[i] = errors[i];
   
  // upDateMotorSpeed(desiredRateThrottle, roll, yaw, pitch);
  prevTime = currTime;
}

void getMPUData(){
  // Read normalized values 
  Vector normGyro = mpu.readNormalizeGyro();

  // Calculate Pitch & Roll
  currAngles[0] = -(atan2(normGyro.XAxis, sqrt(normGyro.YAxis*normGyro.YAxis + normGyro.ZAxis*normGyro.ZAxis))*180.0)/M_PI;
  currAngles[1] = (atan2(normGyro.YAxis, normGyro.ZAxis)*180.0)/M_PI;
  currAngles[2] = atan2(normGyro.YAxis, normGyro.XAxis);


}

double computePID(double error, double prev_error, double kp, double ki, double kd, double time_error){
  double p = kp * error;
  double i = ki * ((error + prev_error) * time_error)/2;
  double d = kd * (error - prev_error)/time_error;

  return (p + i + d);
}

void upDateMotorSpeed(double thrust, double roll, double yaw, double pitch){
  int motor1 = thrust + yaw - pitch - roll;
  int motor2 = thrust - yaw - pitch + roll;
  int motor3 = thrust + yaw + pitch + roll;
  int motor4 = thrust - yaw + pitch - roll;
  Serial.println(motor1);
  Serial.println(motor2);
  Serial.println(motor3);
  Serial.println(motor4);
  Serial.println("");
  delay(1000);
  Motor1.writeMicroseconds(motor1);
  Motor2.writeMicroseconds(motor2);
  Motor3.writeMicroseconds(motor3);
  Motor4.writeMicroseconds(motor4);
}

void getReceiverData(){
  channel[0] = pulseIn(channel_pin1, HIGH);
  channel[1] = pulseIn(channel_pin2, HIGH);
  channel[2] = pulseIn(channel_pin3, HIGH);
  channel[3] = pulseIn(channel_pin4, HIGH);
  channel[4] = pulseIn(channel_pin5, HIGH);
  channel[5] = pulseIn(channel_pin6, HIGH);
}