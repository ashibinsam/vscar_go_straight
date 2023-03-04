int16_t getRotationZ() {
    uint8_t buffer[14];
  I2Cdev::readBytes(MPU6050_ADDR, MPU6050_RA_GYRO_ZOUT_H, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
  }
  //routine from MPU6050.cpp
  
    
  void calibration()
 {
  //Here we do 100 readings for gyro sensitiv Z-axis output -gyroZ, we sum them and divide them by 100.
  // gyrZ0 mean value of gyroZ raw register values in 100 ms
  
  unsigned short times = 100; //Sampling times
  for (int i = 0; i < times; i++)
  {
    gyroZ = getRotationZ();     // gyroZ - Raw register values gyroscope Z axis
    gyroZ0 += gyroZ; //sum all measured values gyroZ
  }
  gyroZ0 /= times; 
  }

 void calcYaw(){
  
  unsigned long currentTime = millis();   //current time(ms)
  //get the current timestamp in Milliseconds since epoch time which is
  dt = (currentTime - lastTime) / 1000.0; //Differential time(s)
  lastTime = currentTime;                 //Last sampling time(ms)

  gyroZ = getRotationZ();
  
  float angularZ = (gyroZ - gyroZ0) / 131.0 * dt; //angular z: =t
  if (fabs(angularZ) < 0.05) //
  {
    angularZ = 0.00;
  }
  gyroAngleZ += angularZ; //returns the absolute value of the z-axis rotazion integral 
  yaw = - gyroAngleZ;

  /*
  Serial.print(" | GyZo = "); Serial.print(toStr(gyroZ0));
  Serial.println();
  Serial.print(" | GyroAngle = "); Serial.print (gyroAngleZ);
  Serial.println();
  */
   }

   //++++++++++++++++++++++++
   
 
  void driveStraight(int speed){
  static unsigned long onTime;
   //to compute corrSpeed(A/B) The yaw data is acquired at the first startup, and the data is updated every ten millisecundes.
  if (millis() - onTime > 10){
  corrSpeed(speed);
  onTime = millis();  
 }  
  }
  
  void corrSpeed(int myspeed){
  calcYaw();
  int  kp= 15; ////Add proportional constant - p( ???)
  //if drive direktion changes: corrSpeedA = mySpeedA - (yawOld - yaw) * kp;
  corrSpeedR = mySpeed + (yaw-yawOld)*kp; //maintain speed by speeding up right motor
  if (corrSpeedR > highspeed)
  {
    corrSpeedR = highspeed;
  }
  else if (corrSpeedR < lowspeed)
  {
    corrSpeedR = lowspeed;
  }
  //if drive direktion changes:corrSpeedB = mySpeedB + (yawOld - yaw) * kp;
  corrSpeedL = mySpeed - (yaw-yawOld)*kp; //if error is positive, slow down left motor
  if (corrSpeedL > highspeed)
  {
    corrSpeedL = highspeed;
  }
  else if (corrSpeedL < lowspeed)
  {
    corrSpeedL = lowspeed;
  }
   
 }


   //*****
   void forward(int is_speed){ 
  driveStraight(is_speed);

   motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  motor1.setSpeed(corrSpeedR);
  motor2.setSpeed(corrSpeedR);
  motor3.setSpeed(corrSpeedL);
  motor4.setSpeed(corrSpeedL);
  
}

void back(int is_speed){
  corrSpeed(is_speed);

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  motor1.setSpeed(corrSpeedL);
  motor2.setSpeed(corrSpeedL);
  motor3.setSpeed(corrSpeedR);
  motor4.setSpeed(corrSpeedR);
}

void left(int is_speed){
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
    motor1.setSpeed(is_speed);
    motor2.setSpeed(is_speed);
    motor3.setSpeed(is_speed);
    motor4.setSpeed(is_speed);
    
}
void right(int is_speed, int turnSpeed, int rate){

  motor3.run(FORWARD);
  motor4.run(FORWARD);
  static unsigned long onTime6;

  if (millis() - onTime6 > 10){
      if(mySpeed < is_speed)
        mySpeed += rate;
      if(mySpeed > is_speed)
        mySpeed -= rate;
      if(tSpeed < turnSpeed)
        tSpeed += rate;
      if(tSpeed > turnSpeed)
        tSpeed -= rate;
      onTime6 = millis(); 
      motor3.setSpeed(mySpeed);
      motor4.setSpeed(mySpeed);
      if(turnSpeed ==255) {
        motor1.run(BACKWARD);
        motor2.run(BACKWARD);
        motor1.setSpeed(mySpeed-tSpeed);
        motor2.setSpeed(mySpeed-tSpeed);

      }
      else {
        motor1.run(FORWARD);
        motor2.run(FORWARD);
        motor1.setSpeed(mySpeed-tSpeed);
        motor2.setSpeed(mySpeed-tSpeed); 
      }
  }
  

}

void left(int is_speed, int turnSpeed, int rate){

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  static unsigned long onTime5;

  if (millis() - onTime5 > 10){
      if(mySpeed < is_speed)
        mySpeed += rate;
      if(mySpeed > is_speed)
        mySpeed -= rate;
      if(tSpeed < turnSpeed)
        tSpeed += rate;
      if(tSpeed > turnSpeed)
        tSpeed -= rate;
      onTime5 = millis(); 
      motor1.setSpeed(mySpeed);
      motor2.setSpeed(mySpeed);
      if(turnSpeed ==255) {
        motor3.run(BACKWARD);
        motor4.run(BACKWARD);
        motor3.setSpeed(mySpeed);
        motor4.setSpeed(mySpeed);

      }
      else {
        motor3.run(FORWARD);
        motor4.run(FORWARD);
        motor3.setSpeed(mySpeed-tSpeed);
        motor4.setSpeed(mySpeed-tSpeed); 
      }
  }
  

}

void halt(){
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void accel(int rate,  int maxSpeed){
  static unsigned long onTime1;
  if (millis() - onTime1 > 10){
  if(mySpeed < maxSpeed)
    mySpeed += rate;
  if(mySpeed > maxSpeed)
    mySpeed -= rate;
  forward(mySpeed);
  onTime1 = millis();  
 }  
  }

void accelRev(int rate,  int maxSpeed){
  static unsigned long onTime2;
   //to compute corrSpeed(A/B) The yaw data is acquired at the first startup, and the data is updated every ten millisecundes.
  if (millis() - onTime2 > 10){
  if(mySpeed < maxSpeed)
    mySpeed += rate;
  if(mySpeed > maxSpeed)
    mySpeed -= rate;
  back(mySpeed);
  onTime2 = millis();  
 }  
  }

void deAccel(int rate,  int minSpeed){
  static unsigned long onTime3;
   //to compute corrSpeed(A/B) The yaw data is acquired at the first startup, and the data is updated every ten millisecundes.
  if (millis() - onTime3 > 10){
  mySpeed -= rate;
  if(mySpeed < minSpeed)
    mySpeed = minSpeed;
  forward(mySpeed);
  onTime3 = millis();  
 }  
  }

void deAccelRev(int rate,  int minSpeed){
  static unsigned long onTime4;
   //to compute corrSpeed(A/B) The yaw data is acquired at the first startup, and the data is updated every ten millisecundes.
  if (millis() - onTime4 > 10){
  mySpeed -= rate;
  if(mySpeed < minSpeed)
    mySpeed = minSpeed;
  back(mySpeed);
  onTime4 = millis();  
 }  
  }
/*
  LeftForward,  
  LeftBackward, 
  RightForward, 
  RightBackward,
  */

void receiveData() {
  if (Serial.available() > 0) {
    // Read the incoming serial data
    String receivedData = Serial.readStringUntil('\n');

    // Parse the string to extract the integer array
    int commaIndex = 0;
    int arrayIndex = 0;
    while (commaIndex >= 0 && arrayIndex < 5) {
      commaIndex = receivedData.indexOf(",");
      if (commaIndex >= 0) {
        String dataString = receivedData.substring(0, commaIndex);
        receivedData = receivedData.substring(commaIndex + 1);
        dataArr[arrayIndex] = dataString.toInt();
        arrayIndex++;
      }
    }

    // Print the received integer array
    for(int i=0; i<5; i++) {
      Serial.print(dataArr[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
