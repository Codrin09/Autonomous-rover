#include <PID_v1.h>
#include <RPLidar.h>
#include <SoftwareSerial.h>

#define RXd 10
#define TXx 11

SoftwareSerial btSerial(RXd, TXx); // RX, TX
float get_travel_distance();
#include "Lidar.h"

unsigned short oldX, oldY, oldTh;

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

//0.2 0.7
static const double coeffP = 0.1;
static const double coeffI = 1.5;
static const double coeffD = 0.0;

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
  Serial.println("start");

  btSerial.begin(115200);
  //Flush all outdated messages
  while(btSerial.available() > 0){
    btSerial.read();
  }

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

  oldX = oldY = 500;
  oldTh = 0;
}

void loop(){
  check_bt();
}


void check_bt(){
  if(btSerial.available() > 0){
    Serial.println("Incomming data on BT");

    String incomingData = btSerial.readString();
    Serial.println(incomingData);
    float trip_distance = 0;
    bool running;
    switch(incomingData.toInt()){
      case 1:
        btSerial.println("Powering motors controlled by PID");
        
        for(int i = 0 ; i < 3; i++){
          scanStartTime = millis();
          while(check_lidar());
          send_readings();
        }

        oldMotorTime = micros();
        while(power_by_PID(500));
        stopMotors();

        btSerial.println("Get position");
        while(true){
          if(btSerial.available() > 0){
            String input = btSerial.readString();
            if(input.toInt() == 1)
              break;
          }
        }
        scanStartTime = millis();
        while(check_lidar());
        send_readings();
        //!REMEMBER TO RESET TRAVEL DISTANCE
        reset_travel_distance();

        for(int i = 0 ; i < 2; i++){
          scanStartTime = millis();
          while(check_lidar());
          send_readings();
        }
        
        btSerial.println("Finish operation 1");
        break;
      case 2:
        btSerial.println("Powering motors at maximum speed without PID");
        power_no_PID(1000, 255);

        break;
      case 3:
        btSerial.println("Turn 45 degrees anti clockwise");
        rotate(45, 1);
        break;
      case 4:
        btSerial.println("Turn 90 degrees clockwise");
        rotate(90, 1);
        break;
      case 5:
        btSerial.println("Turn 45 degrees clockwise");
        rotate(45, 0);
        break;
      case 6:
        btSerial.println("Turn 90 degrees clockwise");
        rotate(90, 0);
        break;
      case 7:
        btSerial.println("Testing");
        while(true){
          while(btSerial.available()){
            String testData = btSerial.readString();
            btSerial.println(testData);
          }
        }
      case 8:
        btSerial.println("Start scanning and mapping");

        running = true;
        while(running){
          scanStartTime = millis();
          while(check_lidar());
          send_readings();
          if(btSerial.available() > 0){
            btSerial.readString();
            running = false;
          }
        }
        btSerial.println("Stop scanning");

        break;
      default:
        btSerial.println("No valid command given");
        break;
    }
    reset_motors();
  }
}

bool power_by_PID(int dist){
  for(int i = 0; i < noOfMotors; i++){
    Setpoint[i] = desiredSpeed - speedScalar[i];
  }

  update_speed();

  if(get_travel_distance() > dist && dist != 0)
    return false;
  return true;
}

float update_speed(){
  newMotorTime = micros();
  if(newMotorTime - oldMotorTime >= 100000){
    float total_spid = 0;
    float totalTime = (newMotorTime - oldMotorTime) * 1.0 / 1000000.0;

    // btSerial.println(String(impulses[0]) + " "+ String(impulses[1]) + " " + String(impulses[2]) + " " + String(impulses[3]));
    // btSerial.println(newMotorTime - oldMotorTime);

    //Impulses per 100ms
    Input[0] = Input[2] = (impulses[0] + impulses[2]) * 1.0 / (2 * totalTime);
    Input[1] = Input[3] = (impulses[1] + impulses[3]) * 1.0 / (2 * totalTime);

    // btSerial.println(String(Input[0]) + " "+ String(Input[1]));
    // print_encoder_distances();

    for(int i = 0; i < noOfMotors; i++){
      spid[i] = 60 * (Input[i] * 3.0 / 1000) / totalTime;
      total_spid += spid[i];
      
      impulses[i] = 0;
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
  while(update_speed() != 0.0);
  power_off_motors();
}

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

float get_travel_distance(){
  float dist = (distance[0] + distance[1] + distance[2] + distance[3]) / 4;
  // btSerial.println((distance[0] - distance[1] + distance[2] - distance[3]));
  return dist;
}

void reset_travel_distance(){
  for(int i = 0; i < noOfMotors; i++)
    distance[i] = 0;
}

void power_off_motors(){
  btSerial.println("Powering off motors");
  for(int i = 0; i < noOfMotors; i++){
    analogWrite(motorControl[2*i], 0);
  }
}

void reset_motors(){
  for(int i = 0; i < noOfMotors; i++){
    distance[i] = 0;
    impulses[i] = 0;
  }
}

void rotate(int deg, int anti_clock){
  //impulses for 90 deg rotation
  const int quarterCircle = 472.5;

  int start_reverse;
  if(anti_clock == 1){
    start_reverse = 1;
  } 
  else{
    start_reverse = 3;
  }
  digitalWrite(motorControl[start_reverse], HIGH);
  digitalWrite(motorControl[start_reverse + 4], LOW);

  double percentage = deg * 1.0 / 90;

  for(int i = 0; i < noOfMotors; i++){
    analogWrite(motorControl[2 * i],255);
  }
  

  int val = quarterCircle * percentage;

  while(impulses[0] < val || impulses[1] < val || impulses[2] < val || impulses[3] < val);

  digitalWrite(motorControl[start_reverse], LOW);
  digitalWrite(motorControl[start_reverse + 4], HIGH);

  power_off_motors();
  reset_travel_distance();
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

void interM0(){impulses[0]++; distance[0] += distOnImpulse;}
void interM1(){impulses[1]++; distance[1] += distOnImpulse;}
void interM2(){impulses[2]++; distance[2] += distOnImpulse;}
void interM3(){impulses[3]++; distance[3] += distOnImpulse;}