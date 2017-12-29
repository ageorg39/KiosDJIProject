
#include <Wire.h> 

#include "I2C.h"
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

char input[10];
int roll,pitch,yaw,throttle,mode,gear;
#define NLCODE 15000

#include <Servo.h>
Servo myservo;

void setup(){
  Serial.begin(115200); //Opens serial connection at 9600bps.     
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
  
  myservo.attach(A0);
  myservo.write(90);
}

int getNum(){
   int c;
   char in;
   c=0;
   
   unsigned long StartTime = millis();
   while(true) {
   
     if((millis()-StartTime)>350)
      break;
     
     if(Serial.available()!=0){
      in=Serial.read();
      //Serial.print(in);
      if(in==10)
        return NLCODE;
      else if(in!=32){
        input[c]=in;
        c++;
      }
      else
        break;
     }
     }
   input[c]='\0';
   
   return atoi(input);
}

unsigned long StartTime = millis();
unsigned long LidarTime = millis();

void loop(){
  // Write 0x04 to register 0x00
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
  }

  byte distanceArray[2]; // array to store distance bytes from read function
  
  // Read 2byte distance from register 0x8f
  nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    delay(1); // Wait 1 ms to prevent overpolling
  }
  unsigned int distance = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  
  // Print Distance
  
  if((millis()-LidarTime)>300){
    LidarTime = millis();
    
    if(distance<30)
      Serial.println(0);
    else
      Serial.println(distance);
  }
  
   roll=getNum();
   pitch=getNum();
   yaw=getNum();
   throttle=getNum();
   mode=getNum();
   gear=getNum();
   
   if(getNum()!=NLCODE){
    StartTime = millis();
     while(getNum()!=NLCODE){
       if((millis()-StartTime)>350)
         break;
     }
   }
     
  if(gear==-10000)
    myservo.write(150);
  else
    myservo.write(90);
    
    delay(50);

}

