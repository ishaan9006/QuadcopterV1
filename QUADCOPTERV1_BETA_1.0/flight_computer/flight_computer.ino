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

#define PITCH 0
#define ROLL 1
#define YAW 2

float currTime = 0;
float prevTime = 0;
float timeError = 0;

double Kp[3] = {1, 1, 1};
double Ki[3] = {1, 1, 1};
double Kd[3] = {1, 1, 1};


int channel[6] = {0, 0, 0, 0, 0, 0};

double errors[3] = {0, 0, 0};     // errors in the order: PITCH ROLL YAW
double prevErrors[3] = {0, 0, 0};     // Previous errors in the order: PITCH ROLL YAW
double integrals[3] = {0, 0, 0};

MPU6050 mpu;

void setup() {

  Serial.begin(9600);   
  Serial.println("Initializing MPU6050");
  Wire.begin();
  mpu.begin();

  Motor1.attach(5, 1000, 2000);
  Motor2.attach(6, 1000, 2000);
  Motor3.attach(9, 1000, 2000);
  Motor4.attach(10, 1000, 2000);
  
}

void loop() {
  currTime = millis();
  timeError = currTime - prevTime;

  getReceiverData();

  double desiredRatePitch    = 0.15 * (channel[CHANNEL3] - 1500);
  double desiredRateRoll     = 0.15 * (channel[CHANNEL4] - 1500);
  double desiredRateYaw      = 0.15 * (channel[CHANNEL2] - 1500);
  double desiredRateThrottle = channel[CHANNEL1];

  // Read normalized values 
  Vector normAccel = mpu.readNormalizeAccel();

  // Calculate Pitch & Roll
  int currPitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  int currRoll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
  int currYaw = 0;


  errors[0] = desiredRatePitch - currPitch;
  errors[1] = desiredRateRoll - currRoll;
  errors[2] = desiredRateYaw - currYaw;




  // // Output
  // Serial.print(" Pitch = ");
  // Serial.print(pitch);
  // Serial.print(" Roll = ");
  // Serial.print(roll);
  
  // Serial.println();
  
  // delay(100);

  double roll   = computePID(errors[ROLL], prevErrors[ROLL], integrals[ROLL], Kp[ROLL], Ki[ROLL], Kd[ROLL], timeError);
  double yaw    = computePID(errors[YAW], prevErrors[YAW], integrals[YAW], Kp[YAW], Ki[YAW], Kd[YAW], timeError);
  double pitch  = computePID(errors[PITCH], prevErrors[PITCH], integrals[PITCH], Kp[PITCH], Ki[PITCH], Kd[PITCH], timeError);


  integrals[PITCH] += errors[PITCH];
  integrals[ROLL]  += errors[ROLL];
  integrals[YAW]   += errors[YAW];

  for(int i=0;i<3;i++) prevErrors[i] = errors[i];
   



  upDateMotorSpeed(desiredRateThrottle, roll, yaw, pitch);
  prevTime = currTime;
}

void getGyroSignals(){
 
}

double computePID(double error, double prev_error, double integral, double kp, double ki, double kd, double time_error){
  double p = kp * error;
  double i = ki * integral;
  double d = kd * (error - prev_error)/time_error;

  return (p + i + d);
}

void upDateMotorSpeed(double thrust, double roll, double yaw, double pitch){
  int motor1 = thrust + yaw - pitch - roll;
  int motor2 = thrust - yaw - pitch + roll;
  int motor3 = thrust + yaw + pitch + roll;
  int motor4 = thrust - yaw + pitch - roll;

  Motor1.writeMicroseconds(motor1);
  Motor2.writeMicroseconds(motor2);
  Motor3.writeMicroseconds(motor3);
  Motor4.writeMicroseconds(motor4);
}

void getReceiverData(){
  
}