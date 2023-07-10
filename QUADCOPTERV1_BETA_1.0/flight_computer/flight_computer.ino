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


float Kp[3] = {1, 1, 1};
float Ki[3] = {1, 1, 1};
float Kd[3] = {1, 1, 1};


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


  upDateMotorSpeed();
  prevTime = currTime;
}

double computePID(double error, double prev_error, double integral, double kp, double ki, double kd){
  double p = kp * error;
  double i = ki * integral;
  double d = kd * (error - prev_error);

  return (p + i + d);
}

void upDateMotorSpeed(){

}







double computePID(double error, double prev_error, double integral, double kp, double ki, double kd) {
  double p = kp * error;
  double d = kd * (error - prev_error);
  double i = ki * integral;

  return p + d + i;
}





















