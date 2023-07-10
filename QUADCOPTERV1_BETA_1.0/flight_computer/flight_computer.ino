#include <Wire.h>

#define PITCH 0
#define ROLL 1
#define YAW 2

// Defining motor pins
#define Motor1 2
#define Motor2 3
#define Motor3 4
#define Motor4 5

float currTime = 0;
float prevTime = 0;
float timeError = 0;

double Kp[3] = {1, 1, 1};
double Ki[3] = {1, 1, 1};
double Kd[3] = {1, 1, 1};

double errors[3] = {0, 0, 0};     // errors in the order: PITCH ROLL YAW
double prevErrors[3] = {0, 0, 0};     // Previous errors in the order: PITCH ROLL YAW
double integrals[3] = {0, 0, 0};


void setup() {

  Serial.begin(9600);   

  Wire.begin();                           //begin the wire comunication  
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
}

void loop() {
  currTime = millis();
  timeError = currTime - prevTime;

  double roll  = computePID(errors[ROLL], prevErrors[ROLL], integrals[ROLL], Kp[ROLL], Ki[ROLL], Kd[ROLL], timeError);
  double yaw   = computePID(errors[YAW], prevErrors[YAW], integrals[YAW], Kp[YAW], Ki[YAW], Kd[YAW], timeError);
  double pitch = computePID(errors[PITCH], prevErrors[PITCH], integrals[PITCH], Kp[PITCH], Ki[PITCH], Kd[PITCH], timeError);


  integrals[PITCH] += errors[PITCH];
  integrals[ROLL]  += errors[ROLL];
  integrals[YAW]   += errors[YAW];

  upDateMotorSpeed();
  prevTime = currTime;
}

void getGyroSignals(){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);

  int16_t GyroX = Wire.read()<<8 | Wire.read();
  int16_t GyroY = Wire.read()<<8 | Wire.read();
  int16_t GyroZ = Wire.read()<<8 | Wire.read();
}

double computePID(double error, double prev_error, double integral, double kp, double ki, double kd, double time_error){
  double p = kp * error;
  double i = ki * integral;
  double d = kd * (error - prev_error)/time_error;

  return (p + i + d);
}

void upDateMotorSpeed(){
  // motor1 = thrust + yaw - pitch - roll
  // motor2 = thrust - yaw - pitch + roll
  // motor3 = thrust + yaw + pitch + roll
  // motor4 = thrust - yaw + pitch - roll
}

void getReceiverData(){
  

}