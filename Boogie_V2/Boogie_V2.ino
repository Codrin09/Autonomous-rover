#include <PID_v1.h>
#include <RPLidar.h>

#define RXd 10
#define TXx 11
#define resetPin 52

float get_travel_distance();

#include "Gyro.h"
#include "Lidar.h"

const byte interruptPin = 2;

float distOnImpulse = 6.0 * PI * 30 / 1000.0;
volatile long impulses[] = {0, 0, 0, 0};
unsigned long newMotorTime, oldMotorTime = 0;

float motor_distance[] = {0, 0, 0, 0};
float spid[] = {0, 0, 0, 0};
int direction[] = {0, 0 , 1, 1};

double Input[4], Output[4];
//450 444 450 450
double desiredSpeed = 500;
int speedScalar[4];
double Setpoint[] = {desiredSpeed, desiredSpeed, desiredSpeed, desiredSpeed};

//0.2 0.7
static const double coeffP = 0.1;
static const double coeffI = 1.5;
static const double coeffD = 0.0;

volatile long lastPrintTime = 0;

int noOfMotors = 2;

PID motor_pid[]={
  PID(&Input[0], &Output[0], &Setpoint[0], coeffP, coeffI, coeffD, DIRECT),
  PID(&Input[1], &Output[1], &Setpoint[1], coeffP, coeffI, coeffD, DIRECT),
  PID(&Input[2], &Output[2], &Setpoint[2], coeffP, coeffI, coeffD, DIRECT),
  PID(&Input[3], &Output[3], &Setpoint[3], coeffP, coeffI, coeffD, DIRECT)
};

//Init method for motors pid
void setPid(PID &motor_pid, int power){
  motor_pid.SetMode(MANUAL);
  analogWrite(power, 0);
  motor_pid.SetMode(AUTOMATIC);
}

void (*interImpulse[4])() = {
  interM0, interM1, interM2, interM3
};

void setEncoders(){
  for(int pin = 18; pin < 18 + noOfMotors; pin++){
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), interImpulse[pin - 18], CHANGE);
  }
}

int motorControl[8];
void setMotorControl(){
  //motors 1-4 DIRECTION
  for(int pin = 42; pin < 42 + noOfMotors; pin++){
    pinMode(pin, OUTPUT);
    motorControl[2 * (pin - 42) + 1] = pin;
    digitalWrite(pin, direction[pin - 42]);
  }

  //motors 1-4 PWM
  for(int pin = 4; pin < 4 + noOfMotors; pin++){
    pinMode(pin, OUTPUT);
    motorControl[2 * (pin - 4)] = pin;
  }
}

void setup(){ 
  Serial3.begin(115200);
  lidar.begin(Serial3);
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  Serial.begin(115200);
  Serial.println("start");

  Serial2.begin(115200);
  //Flush all outdated messages
  while(Serial2.available() > 0){
    Serial2.read();
  }

  digitalWrite(resetPin, HIGH);
  delay(50);
  pinMode(resetPin, OUTPUT);

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

  setup_gyro();
  Serial2.println("RoboBee is ready!");
}

void loop(){
  check_bt();

  //! New add for gyro
  accelgyro.resetFIFO();
}

void check_bt(){
  while(Serial2.available() > 0){
    Serial.println("Incomming data on BT");

    String incomingData = Serial2.readString();

    //Check for lost bytes
    if(!valid_message(incomingData))
      continue;
    incomingData.remove(incomingData.length() - 1);

    run_boogie(incomingData);
    //debug(incomingData);
  }

  // get_gyro(true);
}

//Check if received command is valid(no lost bytes)
bool valid_message(String message){
  int x = 0;
    for(int i = 0; i < message.length(); i++){
      x = x ^ message[i];
    }
    if(x != 0){
      Serial2.println("Send again");
      return false;
    }
    
    Serial2.println("Success");
    return true;
}

//Turn received message into data
void message_to_cmd(String incomingData, String *cmd){    
  incomingData += " ";
  char charData[incomingData.length()];
  incomingData.toCharArray(charData, incomingData.length());

  char* token = strtok(charData, " ");
  int index = 0;
  while(token != 0){
    cmd[index++] = token;
    token = strtok(0, " ");
  }

  free(token);
}

//Execute received command
bool execute_cmd(String *cmd, String* output){
  bool stopping = false;
  int dir = 0;
  int rotation;
  switch((int) cmd[0][0]){
    //rotate
    case (int)'r':
      if(cmd[2].equals("t"))
        dir = 1;
      else 
        dir = 0;
      rotation = rotate(cmd[1].toFloat(), dir);
      *output = "Finish rotate " + String(rotation);
      break;

    //move
    case (int)'m':
      if(!power_by_PID(cmd[1].toInt(), cmd[2].toInt())){
        *output = "Distance:" + String(get_travel_distance()) + ":0";
        return false;
      }
      *output = "Distance:" + String(get_travel_distance())+ ":1";
      break;

    //update landmarks
    case (int)'l':
    Serial.println(cmd[1].toInt());
      for(int i = 0 ; i < cmd[1].toInt(); i++){
        scanStartTime = millis();
        while(check_lidar(&stopping, false));
        send_readings();
      }
      *output = "Finish mapping";
      break;
    case (int)'q':
      digitalWrite(resetPin, HIGH);
      delay(100);
      digitalWrite(resetPin, LOW);
      break;
  }
  return true;
}

//Initialise boogie and execute received command
void run_boogie(String incomingData){
  Serial.println(incomingData);
  String *cmd = new String[10];
  String output;
  float old_th;

  if(incomingData.equals("start")){
    Serial.println("starting");
    //Initialise map
    cmd[0] = "l";
    cmd[1] = "3";
    execute_cmd(cmd, &output);

    Serial2.println("Create new path");
  }
  else{
    message_to_cmd(incomingData, cmd);

    //If received move command update gyroscope orientation before executing
    old_th = get_gyro(true);
    bool exec_result = execute_cmd(cmd, &output);

    //!REMEMBER TO RESET MOTORS/TRAVEL DISTANCE
    reset_motors();
  }
  //Send response back
  Serial2.println(output);
  if(cmd[0]=="m"){
    Serial2.println("Orientation:" + String(old_th - get_gyro(false)));
  }
  free(cmd);
}

//Update the PID if enough time passed to gather encoder readings
float update_speed(){
  //!Remember to update gyro to avoid FIFO overflow
  get_gyro(false);

  newMotorTime = micros();
  if(newMotorTime - oldMotorTime >= 100000){
    float total_spid = 0;
    float totalTime = (newMotorTime - oldMotorTime) * 1.0 / 1000000.0;

    //Approximate impulses per second from 100ms sample
    Input[0] = Input[2] = (impulses[0] + impulses[2]) * 1.0 / (2 * totalTime);
    Input[1] = Input[3] = (impulses[1] + impulses[3]) * 1.0 / (2 * totalTime);

    //Calculate wheel rotations per second
    for(int i = 0; i < noOfMotors; i++){
      spid[i] = 60 * (Input[i] * 3.0 / 1000) / totalTime;
      total_spid += spid[i];
      
      impulses[i] = 0;
      //If enough time passed update PID
      if(motor_pid[i].Compute()){
        analogWrite(motorControl[2*i], round(Output[i]));
      }
    }
    oldMotorTime = newMotorTime;

    return total_spid;
  }
  return -1;
}

void stopMotors(){
  for(int i = 0; i < noOfMotors; i++){
    Setpoint[i] = 0;
  }
  //Wait until wheels have stopped
  while(update_speed() != 0.0);
  power_off_motors();
}

//Power motors just by applying PWM without PID
void power_no_PID(int dist, int PWM){

  for(int i = 0; i < noOfMotors; i++){
    analogWrite(motorControl[2*i], PWM);
  }
  bool moving  = true;
  while(moving){
    bool printed = print_encoder_distances();
    if(printed){
      for(int i = 0; i < noOfMotors; i++){
        impulses[i] = 0;
        motor_distance[i] = 0;
      }
    }

    if(Serial2.available()){
      Serial2.readString();
      moving = false;
    }
  }
  power_off_motors();
}

//Calculate last move distance
float get_travel_distance(){
  float dist = 0;
  for(int i = 0; i < noOfMotors; i++){
    dist += motor_distance[i];
  }
  // Serial2.println((motor_distance[0] - motor_distance[1] + motor_distance[2] - motor_distance[3]));
  return dist / noOfMotors;
}

void power_off_motors(){
  Serial2.println("Powering off motors");
  for(int i = 0; i < noOfMotors; i++){
    analogWrite(motorControl[2*i], 0);
  }
}

//Reset motors after movement
void reset_motors(){
  for(int i = 0; i < noOfMotors; i++){
    motor_distance[i] = 0;
    impulses[i] = 0;
  }
  set_motors_direction(true);
}

//Absolute method because arduino lib abs() can't handle floats :-?
float absolute(float value){
  if(value < 0)
    return -value;
  else
    return value;
}

//Rotate robot by powering a set of motors in one direction and the others in the opposite direction
int rotate(int deg, int anti_clock){
  if(deg == 0)
    return 0;
  //impulses for 90 deg rotation
  const int quarterCircle = 510;

  int start_reverse;
  if(anti_clock == 1){
    start_reverse = 1;
  } 
  else{
    start_reverse = 3;
  }
  digitalWrite(motorControl[start_reverse], HIGH);
  digitalWrite(motorControl[start_reverse + 4], LOW);

  int val = quarterCircle * (deg * 1.0 / 90);
  int gyro_old = get_gyro(true);
  int gyro_new;

  for(int i = 0; i < noOfMotors; i++){
    analogWrite(motorControl[2 * i], 255);
  }

  bool turning = true;
  while(turning){
    get_gyro(false);
    for(int i = 0 ; i < noOfMotors; i++)
      if(impulses[i] >= val){
        turning = false;
        break;
      }
  }
  stopMotors();

  digitalWrite(motorControl[start_reverse], LOW);
  digitalWrite(motorControl[start_reverse + 4], HIGH);
  reset_motors();

  gyro_new = get_gyro(false);

  return absolute(gyro_new - gyro_old);
} 

bool print_encoder_distances(){
  if(millis() - lastPrintTime > 1000){
    float sum = 0;
    String outString = "";
    for(int i = 0; i < noOfMotors; i++){
      if(i % 2 == 0)
        sum += motor_distance[i];
      else
        sum -+ motor_distance[i];
      outString += String(impulses[i]) + " ";
    }
    Serial.println(sum);
    Serial2.println(outString);

    lastPrintTime = millis();
    return true;
  }
  return false;
}

//Methods for encoder interrupts
void interM0(){impulses[0]++; motor_distance[0] += distOnImpulse;}
void interM1(){impulses[1]++; motor_distance[1] += distOnImpulse;}
void interM2(){impulses[2]++; motor_distance[2] += distOnImpulse;}
void interM3(){impulses[3]++; motor_distance[3] += distOnImpulse;}

bool power_by_PID(int dist, int dir){
  set_motors_direction(bool(dir));

  for(int i = 0; i < noOfMotors; i++){
    Setpoint[i] = desiredSpeed - speedScalar[i];
  }
  
  bool stopping = false;
  while(true){
    update_speed();
    check_lidar(&stopping, true);
    if(stopping){
      return false;
    }

    if(get_travel_distance() > dist && dist != 0){
      stopMotors();
      return true;
    }
  } 
}

bool set_motors_direction(bool dir){
  for(int i = 0 ; i < noOfMotors; i++){
    if(!dir)
      digitalWrite(motorControl[2 * i + 1],1 - direction[i]);
    else
      digitalWrite(motorControl[2 * i + 1], direction[i]);
  }
}