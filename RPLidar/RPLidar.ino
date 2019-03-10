#include <RPLidar.h>
#include <SoftwareSerial.h>

#define RXd 10
#define TXx 11

SoftwareSerial btSerial(RXd, TXx); // RX, TX
RPLidar lidar;

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal        
bool wasSet[361];
unsigned short data[361];

volatile unsigned long scanStartTime;
bool toPrint = false;

void setup() {
  // bind the RPLIDAR driver to the arduino hardware serial
  Serial3.begin(115200);
  lidar.begin(Serial3);

  Serial.begin(115200);
  btSerial.begin(115200);

  //Flush all outdated messages
  while(btSerial.available() > 0){
    btSerial.read();
  }
  
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
}
bool wait = true;
void loop() {
  while(wait){
    check_bt();
  }

  if(toPrint){
    int errors = 0;
    for(int i = 0 ; i < 360; i++){
      btSerial.println(String(i) + " (" + String(wasSet[i]) + "): " + String(data[i]));
      if(!wasSet[i])
        errors++;

      wasSet[i] = false;
      data[i] = 0;
    }

    scanStartTime = millis();
    toPrint = false;
    // Serial.println(errors);
  }

  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    // read for 3s
    if(millis() - scanStartTime < 2000){ 
      // Serial.print(",");
    if(quality > 8 && distance < 2000){
        wasSet[(int) round(angle)] = true;
        data[(int) round(angle)] = int(distance);
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
      scanStartTime = millis();
      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, 255);

      delay(1000);
    }
  }
  check_bt();
}

void check_bt(){
  if(btSerial.available() > 0){
    String incomingData = btSerial.readString();
    switch(incomingData.toInt()){
      case 1:
        Serial.println("Start scanning and mapping");
        scanStartTime = millis();
        wait = false;
        break;
      default:
        Serial.println("Stop scanning");
        analogWrite(RPLIDAR_MOTOR, 0);
        for(int i = 0 ; i < 360; i++){
          wasSet[i] = false;
          data[i] = 0;
        }
        wait = true;
        break;
    }
  }
}
