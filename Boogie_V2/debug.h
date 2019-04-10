//Method used for debug and testing individual components
void debug(String incomingData){
  float new_th, old_th = 0;
  bool stopping = false;
  bool running = true;
  switch(incomingData.toInt()){
      case 1:
        Serial2.println("Powering motors controlled by PID");
        old_th = get_gyro(true);
        for(int i = 0 ; i < 3; i++){
          scanStartTime = millis();
          while(check_lidar(&stopping, false));
          send_readings();
        }

        oldMotorTime = micros();
        power_by_PID(1000);
        stopMotors();

        Serial2.println("Get position");
        while(true){
          if(Serial2.available() > 0){
            String input = Serial2.readString();
            if(input.toInt() == 1)
              break;
          }
        }

        scanStartTime = millis();
        while(check_lidar(&stopping, false));
        send_readings();
        //!REMEMBER TO RESET TRAVEL DISTANCE
        reset_motors();

        new_th = get_gyro(false);

        Serial2.println("New orientation");
        Serial2.println(old_th - new_th);

        for(int i = 0 ; i < 2; i++){
          scanStartTime = millis();
          while(check_lidar(&stopping, false));
          send_readings();
        }
        
        Serial2.println("Finish operation 1");
        break;
      case 2:
        Serial2.println("Powering motors at maximum speed without PID");
        power_no_PID(1000, 255);

        break;
      case 3:
        Serial2.println("Turn 45 degrees anti clockwise");
        for(int i = 0 ; i < 3; i++){
          scanStartTime = millis();
          while(check_lidar(&stopping, false));
          send_readings();
        }

        rotate(45, 1);

        Serial2.println("Rotate");
        Serial2.println("Get position");
        while(true){
          if(Serial2.available() > 0){
            String input = Serial2.readString();
            if(input.toInt() == 1)
              break;
          }
        }

        scanStartTime = millis();
        while(check_lidar(&stopping, false));
        send_readings();
        //!REMEMBER TO RESET TRAVEL DISTANCE
        reset_motors();

        for(int i = 0 ; i < 2; i++){
          scanStartTime = millis();
          while(check_lidar(&stopping, false));
          send_readings();
        }

        break;
      case 4:
        Serial2.println("Turn 90 degrees clockwise");
        rotate(90, 1);
        break;
      case 5:
        Serial2.println("Turn 45 degrees clockwise");
        rotate(45, 0);
        break;
      case 6:
        Serial2.println("Turn 90 degrees clockwise");
        rotate(90, 0);
        break;
      case 7:
        Serial2.println("Testing");
        // Serial2.println("Calculate path");

        while(true){
          if(Serial2.available()){
            String testData = Serial2.readString();

            if(testData == "exit"){
              break;
            }
          }
        }
        Serial2.println("Breaked");
        break;
      case 8:
        Serial2.println("Start scanning and mapping");

        running = true;
        while(running){
          // update_gyro();
          scanStartTime = millis();
          while(check_lidar(&stopping, false));
          send_readings();
          if(Serial2.available() > 0){
            Serial2.readString();
            running = false;
          }
        }
        Serial2.println("Stop scanning");

        break;
      default:
        Serial2.println("No valid command given");
        break;
    }
    reset_motors();
}

//0-3 inidividual / 4 for all
void test_motor(int motorNo){
  if(millis() - oldMotorTime >= 1000){
    newMotorTime = millis();
    float totalTime = (newMotorTime - oldMotorTime) * 1.0 / 1000.0;

    for(int i = 0; i < noOfMotors; i++){
      if(motorNo == 4 || i == motorNo){
        analogWrite(motorControl[2*i], 255);
        spid[i] = 60 * (impulses[i] * 3.0 / 1000.0) / totalTime;  

        impulses[i] = 0;
      }
    }
    oldMotorTime = newMotorTime;
  }
}