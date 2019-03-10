#include <PID_v1.h>
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
bool toPrint = false;

const byte interruptPin = 2;
volatile long count = 0;

double distOnImpulse = 6.0 * PI * 30 / 1000.0;
volatile long impulses[] = {0, 0, 0, 0};
unsigned long newMotorTime, oldMotorTime = 0;

double distance[] = {0, 0, 0, 0};
double spid[] = {0, 0, 0, 0};
int direction[] = {0, 0 , 1, 1};

double Input[4], Output[4];
//450 444 450 450
double desiredSpeed = 500;
int speedScalar[4];
double Setpoint[] = {desiredSpeed, desiredSpeed, desiredSpeed, desiredSpeed};

static const double coeffP = 0.2;
static const double coeffI = 0.7;
static const double coeffD = 0;

volatile unsigned long scanStartTime;
volatile long lastPrintTime = 0;
int noOfMotors = 4;

PID motor_pid[]={
  PID(&Input[0], &Output[0], &Setpoint[0], coeffP, coeffI, coeffD, DIRECT),
  PID(&Input[1], &Output[1], &Setpoint[1], coeffP, coeffI, coeffD, DIRECT),
  PID(&Input[2], &Output[2], &Setpoint[2], coeffP, coeffI, coeffD, DIRECT),
  PID(&Input[3], &Output[3], &Setpoint[3], coeffP, coeffI, coeffD, DIRECT)
};

void setPid(PID &motor_pid, int power){
  motor_pid.SetMode(MANUAL);
  analogWrite(power, 0);
  motor_pid.SetMode(AUTOMATIC);
}


void (*interImpulse[4])() = {
  interM0, interM1, interM2, interM3
};
void setEncoders(){
  for(int pin = 18; pin < 22; pin++){
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), interImpulse[pin - 18], CHANGE);
  }
}

int motorControl[8];
void setMotorControl(){
  //motors 1-4 for DIR
  for(int pin = 42; pin < 46; pin++){
    pinMode(pin, OUTPUT);
    motorControl[2 * (pin - 42) + 1] = pin;
    digitalWrite(pin, direction[pin - 42]);
  }

  // digitalWrite(44, 1);
  // digitalWrite(45, 0);

  //motors 1-4 for PWM
  for(int pin = 4; pin < 8; pin++){
    pinMode(pin, OUTPUT);
    motorControl[2 * (pin - 4)] = pin;
  }
}

void setup(){ 
  Serial3.begin(115200);
  lidar.begin(Serial3);
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  Serial.begin(115200);
  Serial.println("start");                // a personal quirk

  btSerial.begin(115200);

  setEncoders();
  setMotorControl();
  
  for(int i = 0; i < noOfMotors; i++){
    setPid(motor_pid[i], i + 5);
    speedScalar[i] = 0;
  }
  speedScalar[1] = 8;
  speedScalar[3] = 8;
  speedScalar[0] = 4;
  speedScalar[2] = 4;
}

void loop(){
  check_bt();
}

void check_bt(){
  if(btSerial.available() > 0){
      Serial.println("Incomming data on BT");

      String incomingData = btSerial.readString();
      switch(incomingData.toInt()){
        case 1:
          Serial.println("Powering motors controlled by PID");
          btSerial.println("Powering motors controlled by PID");

          // scanStartTime = millis();
          // while(check_ladar());

          power_by_PID(0);

          // scanStartTime = millis();
          // while(check_ladar());

          break;
        case 2:
          Serial.println("Powering motors at maximum speed without PID");
          btSerial.println("Powering motors at maximum speed without PID");
          power_no_PID(1000, 255);

          break;
        case 3:
          Serial.println("Turn 45 degrees anti clockwise");
          btSerial.println("Turn 45 degrees anti clockwise");
          rotate(45, 1);
          break;
        case 4:
          Serial.println("Turn 90 degrees clockwise");
          btSerial.println("Turn 90 degrees clockwise");
          rotate(90, 1);
          break;
        case 5:
          Serial.println("Turn 45 degrees clockwise");
          btSerial.println("Turn 45 degrees clockwise");
          rotate(45, 0);
          break;
        case 6:
          Serial.println("Turn 90 degrees clockwise");
          btSerial.println("Turn 90 degrees clockwise");
          rotate(90, 0);
          break;
        default:
          Serial.println("No valid command given");
          break;
      }

      reset_motors();
    }
}

void power_by_PID(int dist){
  bool stopping = false;
  double total_spid = 0;

  for(int i = 0; i < noOfMotors; i++){
    Setpoint[i] = desiredSpeed - speedScalar[i];
  }

  while(true){
    if(distance[0] >= dist && dist > 0){
      stopMotors(&stopping);
    }

    newMotorTime = millis();
    double total_spid = 0;

    if(newMotorTime - oldMotorTime >= 100){

      double totalTime = (newMotorTime - oldMotorTime) * 1.0 / 1000.0;

      // btSerial.println(String(impulses[0]) + " "+ String(impulses[1]) + " " + String(impulses[2]) + " " + String(impulses[3]));

      //Impulses per 100ms
      Input[0] = Input[2] = (impulses[0] + impulses[2]) * 1.0 / (2 * totalTime);
      Input[1] = Input[3] = (impulses[1] + impulses[3]) * 1.0 / (2 * totalTime);

      // btSerial.println(String(Input[0]) + " "+ String(Input[1]));
      // print_encoder_distances();

      for(int i = 0; i < 4; i++){
        spid[i] = 60 * (Input[i] * 3.0 / 1000.0) / totalTime;
        total_spid += spid[i];
        
        impulses[i] = 0;
        if(motor_pid[i].Compute()){
          analogWrite(motorControl[2*i], (int) Output[i]);
        }
      }
      oldMotorTime = newMotorTime;
    }
    if(btSerial.available()){
      String incomingData = btSerial.readString();
      bool cont = true;
      switch (incomingData.toInt()){
        case 2:
          btSerial.println("Scaling motor 0 speed by: " + String(++speedScalar[0]));
          break;
        case 3:
          btSerial.println("Scaling motor 0 speed by: " + String(--speedScalar[0]));
          break;
        case 4:
          btSerial.println("Scaling motor 1 speed by: " + String(++speedScalar[1]));
          break;
        case 5:
          btSerial.println("Scaling motor 1 speed by " + String(--speedScalar[1]));
          break;
        default:
          cont = false;
          btSerial.println("No valid command received");
          break;
      }
      if(cont)
        continue;
      stopMotors(&stopping);
    }
    if(total_spid == 0 && stopping){
      power_off_motors();
      break;
    }
  }
}


void stopMotors(bool *stopping){
  for(int i = 0; i < noOfMotors; i++){
    Setpoint[i] = 0;
  }
  *stopping = true;
}

bool check_ladar(){
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    // read for 3s
    if(millis() - scanStartTime < 2000){ 
      // Serial.print(",");
      if(quality > 8 && distance < 2000){
        wasSet[(int) int(angle)] = true;
        data[(int) int(angle)] = int(distance);
      }
    }
    else{
      btSerial.println("Sending ladar readings");
      analogWrite(RPLIDAR_MOTOR, 0);
      toPrint = true;
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
      scanStartTime = millis();
      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, 255);

      delay(1000);
    }
  }
  return true;
}

void power_no_PID(int dist, int PWM){

  for(int i = 0; i < 4; i++){
    analogWrite(motorControl[2*i], PWM);
  }
  bool moving  = true;
  while(moving){
    bool printed = print_encoder_distances();
    if(printed){
      for(int i = 0; i < 4; i++){
        impulses[i] = 0;
        distance[i] = 0;
      }
    }

    if(btSerial.available()){
      btSerial.readString();
      moving = false;
    }
  }
  power_off_motors();
}

double get_travel_distance(){
  btSerial.println(distance[0] - distance[1]);
  return distance[0];
}

void power_off_motors(){
  btSerial.println("Powering off motors");
  for(int i = 0; i < noOfMotors; i++){
    analogWrite(motorControl[2*i], 0);
  }
}

void reset_motors(){
  for(int i = 0; i < 4; i++){
    distance[i] = 0;
    impulses[i] = 0;
  }
}

void rotate(int deg, int anti_clock){
  //impulses for 90 deg rotation
  const int quarterCircle = 472;

  int motor1, motor2;
  if(anti_clock == 1){
    motor1 = 0;
    motor2 = 2;
  } 
  else{
    motor1 = 2;
    motor2 = 0;
  }

  double percentage = deg * 1.0 / 90;
  //Switch direction of rotation of motor
  digitalWrite(motorControl[motor1 + 1],1);

  analogWrite(motorControl[motor1],255);
  analogWrite(motorControl[motor2],255);

  int val = quarterCircle * percentage;

  while(impulses[0] < val || impulses[1] < val);

  digitalWrite(motorControl[motor1 + 1],0);
  analogWrite(motorControl[motor1],0);
  analogWrite(motorControl[motor2],0);
}

bool print_encoder_distances(){
    if(millis() - lastPrintTime > 1000){
    Serial.println(distance[0] - distance[1] + distance[2] - distance[3]);

    btSerial.println(String(impulses[0]) + " " + String(impulses[1]) + " " + String(impulses[2]) + " " + String(impulses[3]));

    lastPrintTime = millis();
    return true;
  }
  return false;
}

//0-3 inidividual / 4 for all
void test_motor(int motorNo){
  if(millis() - oldMotorTime >= 1000){
    newMotorTime = millis();
    double totalTime = (newMotorTime - oldMotorTime) * 1.0 / 1000.0;
    // btSerial.println(String(impulses[0]) + " "+ String(impulses[1]) + " " + String(impulses[2]) + " " + String(impulses[3]));

    for(int i = 0; i < 4; i++){
      if(motorNo == 4 || i == motorNo){
        analogWrite(motorControl[2*i], 255);
        spid[i] = 60 * (impulses[i] * 3.0 / 1000.0) / totalTime;  

        impulses[i] = 0;
      }
    }
    oldMotorTime = newMotorTime;
  }
}

void interM0(){impulses[0]++; distance[0] += distOnImpulse;}
void interM1(){impulses[1]++; distance[1] += distOnImpulse;}
void interM2(){impulses[2]++; distance[2] += distOnImpulse;}
void interM3(){impulses[3]++; distance[3] += distOnImpulse;}