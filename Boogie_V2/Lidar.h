bool check_ladar(){
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    // read for 3s
    if(millis() - scanStartTime < 500){ 
      // Serial.print(",");
      if(quality > 8 && distance < 2000){
          wasSet[(int) round(angle)] = true;
          data[(int) round(angle)] = int(distance);
        }
    }
    else{
      return false;
    }
    //perform data processing here... 
    
  } else {
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
  btSerial.println("Sending ladar readings");
  int errors = 0;
  for(int i = 0 ; i < 360; i++){
    btSerial.println(String(i) + " (" + String(wasSet[i]) + "): " + String(data[i]));
    if(!wasSet[i])
      errors++;

    wasSet[i] = false;
    data[i] = 0;
  }
}