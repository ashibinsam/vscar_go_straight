#include "Wire.h"
#include "I2Cdev.h"
#include <AFMotor.h>
#define MPU6050_ADDR 0x68 // Alternatively set AD0 to HIGH  --> Address = 0x69

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
int dataArr[5]={0,0,0,0,0};
int highspeed=255;
  int lowspeed=150;
  int mySpeed = 2;   // speed
  int tSpeed;       //turn Speed
  
  int corrSpeedL;       //Left Motor
  int corrSpeedR;       //Right Motor
  unsigned long t1 = 0;
  unsigned long t2 = 0;

  #define MPU6050_RA_GYRO_ZOUT_H      0x47
  //uint8_t buffer[14];
  float gyroZ; // Raw register values  gyroscope
  
  unsigned long lastTime = 0;
  
  float dt;      //Differential time
  long gyroZ0 = 0;  //Gyro offset = mean value
  float yaw;
  float yawOld = 0;
  float gyroAngleZ = 0; //Angle variable
  
  
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // wake up!
  Wire.endTransmission(true);
  
  delay(2000);
  Serial.print("   ***   straight_line_4 ****");
  
  // Call this function if you need to get the IMU error values for your module
  calibration();

}

void loop() {
  receiveData();
  t1 = millis(); //The millis function returns the number of milliseconds since Arduino board has been powered up.
  t2 = t1;
  mySpeed=100;
  while (abs(t2 - t1)< 1500) {
    //driveStraight();
  accel(10,255);  //go forward
  t2 = millis();
  }
    
  Serial.print("   ***   Yaw:");
  Serial.println(yaw);
  Serial.print("Forward: Speed L ");
  Serial.print(corrSpeedL);
  Serial.print("   |   Speed R:");
  Serial.println(corrSpeedR);  
  Serial.println();
  //yawOld = yaw; 
   
  t1 = millis(); 
  t2 = t1;
  tSpeed = 0;
  while (abs(t2 - t1) < 1500) {
    //driveStraight;
    //go back -inverse
    left(255,250,10);
    t2 = millis();
  }

  // yawOld = yaw;
  // t1 = millis(); 
  // t2 = t1;
  // tSpeed = 0;
  // while (abs(t2 - t1) < 1500) {
  //   //driveStraight;
  //   right(255,250,10);  //go back -inverse
  //   t2 = millis();
  // }
  
  t1 = millis(); //The millis function returns the number of milliseconds since Arduino board has been powered up.
  t2 = t1;
  yawOld = yaw; 
  while (abs(t2 - t1)< 1500) {
    
    //driveStraight();
    accel(50,70);  //go forward
    t2 = millis();
    }
    Serial.print("   ***   Yaw:");
  Serial.println(yaw);
  Serial.print("Forward: Speed L ");
  Serial.print(corrSpeedL);
  Serial.print("   |   Speed R:");
  Serial.println(corrSpeedR);  
  Serial.println();
  yawOld = yaw;  
    

  t1 = millis(); 
  t2 = t1;
  while (abs(t2 - t1) < 1500) {
    halt();
    t2 = millis();
  } 


      
  // t1 = millis(); 
  // t2 = t1;
  // mySpeed=100;
  // while (abs(t2 - t1) < 2500) {
  //   //driveStraight;
  //   accelRev(10,255);  //go back -inverse
  //   t2 = millis();
  // }

  // t1 = millis(); 
  // t2 = t1;
  
  // while (abs(t2 - t1) < 2500) {
  //   //driveStraight;
  //   accelRev(10,30);   //go back -inverse
  //   t2 = millis();
  // }

  // t1 = millis(); 
  // t2 = t1;
  // while (abs(t2 - t1) < 2500) {
  //   halt();
  //   t2 = millis();
  // }


}
