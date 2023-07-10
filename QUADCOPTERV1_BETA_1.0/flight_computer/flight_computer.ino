#include <Wire.h>


float currTime = 0;
float prevTime = 0;

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




  prevTime = currTime;
}
