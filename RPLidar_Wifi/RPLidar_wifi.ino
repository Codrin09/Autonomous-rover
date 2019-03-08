#include <RPLidar.h>
#include "/Users/codrin/Documents/Projects/3rdYear/Boogie_V2/Mega_Wifi.h"
RPLidar lidar;

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal        
bool wasSet[361];
unsigned short data[361];
bool staph = false;

volatile unsigned long lastTime;
bool toPrint = false;
WiFiClient client;

void setup() {
  Serial.begin(115200);

  Serial.println("connecting");
  wifi_connect();
  client = client_connect();
  Serial.println("connected");

  // bind the RPLIDAR driver to the arduino hardware serial
  Serial3.begin(115200);
  lidar.begin(Serial3);
  
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
}

void loop() {

  if(toPrint){
    int errors = 0;
    for(int i = 0 ; i <= 360; i++){
      client.print(i);
      client.print(" (");
      client.print(wasSet[i]);
      client.print("): ");
      client.println(data[i]);
      if(!wasSet[i])
        errors++;

      wasSet[i] = false;
    }
    lastTime = millis();
    toPrint = false;
    // Serial.println(errors);
  }

  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    // read for 3s
    if(millis() - lastTime < 2000){ 
      // Serial.print(",");
      if(quality > 8 && distance < 2000){
        wasSet[round(angle)] = true;
        data[round(angle)] = round(distance);
      }
    }
    else{
      toPrint = true;
    }
    //perform data processing here... 
    
  } else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      // detected...
      lidar.startScan();
      lastTime = millis();
      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, 255);

      delay(1000);
    }
  }
}
