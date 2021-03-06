//Lidar control functions

bool wasSet[360];
unsigned short lidar_data[360];
float travel_distance;

RPLidar lidar;
#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal
volatile unsigned long scanStartTime;

bool check_lidar(bool stopping, bool moving){
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    if(distance < 100){
      stopping = true;
    }
    if(moving)
      return true;

    // read for 500ms
    if(millis() - scanStartTime < 500){ 
      int aprox_angle = round(angle);
      if(quality > 8){
        wasSet[aprox_angle] = true;
        lidar_data[aprox_angle] = int(distance);
      }
    }
    else{
      return false;
    }    
  } 
  else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      // detected...
      lidar.startScan();
      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, 255);

      delay(1000);
    }
    scanStartTime = millis();
  } 
  return true;
}

void send_readings(){
  Serial2.println("Sending lidar readings");

  for(int i = 0 ; i < 360; i++){
    // Serial.println(String(i) + " (" + String(wasSet[i]) + "): " + String(lidar_data[i]) + " : " + String(get_travel_distance()));
    Serial2.println(String(i) + " (" + String(wasSet[i]) + "): " + String(lidar_data[i]) + " : " + String(get_travel_distance()));
    if(!wasSet[i])

    wasSet[i] = false;
    lidar_data[i] = 0;
  }
}