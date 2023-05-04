//#include "Wire.h"
//#include "I2Cdev.h"
#include <AFMotor.h>
//#define MPU6050_ADDR 0x68 // Alternatively set AD0 to HIGH  --> Address = 0x69

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
int dataArr[5]={0,0,0,0,0};
int highspeed=255;
  int lowspeed=160;
  int mySpeed = 100;   // speed
  int tSpeed;       //turn Speed
  
  int corrSpeedL=255;       //Left Motor
  int corrSpeedR=220;       //Right Motor
  unsigned long t1 = 0;
  unsigned long t2 = 0;

 // #define MPU6050_RA_GYRO_ZOUT_H      0x47
  //uint8_t buffer[14];
  float gyroZ; // Raw register values  gyroscope
  
  unsigned long lastTime = 0;
  
  float dt;      //Differential time
  long gyroZ0 = 0;  //Gyro offset = mean value
  float yaw;
  float yawOld = 0;
  float gyroAngleZ = 0; //Angle variable
  
  
void setup() {
  delay(1000);
  Serial.begin(9600);
  //Wire.begin();
 // Wire.beginTransmission(MPU6050_ADDR);
 // Wire.write(0x6B); // PWR_MGMT_1 register
  //Wire.write(0); // wake up!
 // Wire.endTransmission(true);
  
  delay(2000);
  Serial.print("   ***   straight_line_4 ****");
  
  // Call this function if you need to get the IMU error values for your module
  //calibration();

}

void loop() {
  receiveData();
  while (dataArr[0] == 1 && dataArr[1] == 0) {
    //driveStraight();
  accel(dataArr[3],dataArr[2]);  //go forward
  Serial.println("Forward");
  receiveData();
  // printData();
  }
    
  // Serial.print("   ***   Yaw:");
  // Serial.println(yaw);
  // Serial.print("Forward: Speed L ");
  // Serial.print(corrSpeedL);
  // Serial.print("   |   Speed R:");
  // Serial.println(corrSpeedR);  
  // Serial.println();
  //yawOld = yaw; 
   

  tSpeed = 0;
  while (dataArr[0] == 1 && dataArr[1] == -1) {
    //left
    left(dataArr[2],dataArr[4],dataArr[3]);
    Serial.println("Left");
    receiveData();
    // printData();
  }

     //yawOld = yaw;

  tSpeed = 0;
  while (dataArr[0] == 1 && dataArr[1] == 1) {
    //driveStraight;
    right(dataArr[2],dataArr[4],dataArr[3]);  //right
    Serial.println("Right");
    receiveData();
    // printData();
  }

  
  // Serial.print("   ***   Yaw:");
  // Serial.println(yaw);
  // Serial.print("Forward: Speed L ");
  // Serial.print(corrSpeedL);
  // Serial.print("   |   Speed R:");
  // Serial.println(corrSpeedR);  
  // Serial.println();
  //yawOld = yaw;  
    
  while (dataArr[0] == 0) {
    halt();
    receiveData();
    // printData();
    Serial.println("Stop");
  } 


      
  t1 = millis(); 
  
  while (abs(t2 - t1) > 2000) {
    t2 = t1;
    printData();
  }

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
