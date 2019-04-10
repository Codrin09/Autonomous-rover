#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

int16_t ax, ay, az, gx, gy, gz;

double timeStep, timeGyro, timePrev;
double arx, ary, arz, grx, gry, grz, gsx, gsy, gsz, rx, ry, rz;

int i;
double gyroScale = 131;

void setup() {

  Wire.begin();
  Serial.begin(115200);
  accelgyro.initialize();

  timeGyro = millis();

  i = 1;

}

float square(int a, int b){
  float res = 1;
  for(int i = 0 ; i < b; i++)
    res *= a;
   return res;
}

void loop() {

  // set up time for integration
  timePrev = timeGyro;
  timeGyro = millis();
  timeStep = (timeGyro - timePrev) / 1000; // time-step in s

  // collect readings
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // apply gyro scale from datasheet
  gsx = gx/gyroScale;   gsy = gy/gyroScale;   gsz = gz/gyroScale;

  // calculate accelerometer angles
  arx = (180/3.141592) * atan(ax / sqrt(square(ay, 2) + square(az, 2))); 
  ary = (180/3.141592) * atan(ay / sqrt(square(ax, 2) + square(az, 2)));
  arz = (180/3.141592) * atan(sqrt(square(ay) + square(ax)) / az);

  // set initial values equal to accel values
  if (i == 1) {
    grx = arx;
    gry = ary;
    grz = arz;
  }
  // integrate to find the gyro angle
  else{
    grx = grx + (timeStep * gsx);
    gry = gry + (timeStep * gsy);
    grz = grz + (timeStep * gsz);
  }  

  Serial.print(grx);   Serial.print("\t");
  Serial.print(gry);   Serial.print("\t");
  Serial.println(grz);

  // print result
  // Serial.print(i);   Serial.print("\t");
  // Serial.print(timePrev);   Serial.print("\t");
  // Serial.print(time);   Serial.print("\t");
  // Serial.print(timeStep, 5);   Serial.print("\t\t");
  // Serial.print(ax);   Serial.print("\t");
  // Serial.print(ay);   Serial.print("\t");
  // Serial.print(az);   Serial.print("\t\t");
  // Serial.print(gx);   Serial.print("\t");
  // Serial.print(gy);   Serial.print("\t");
  // Serial.print(gz);   Serial.print("\t\t");
  // Serial.print(arx);   Serial.print("\t");
  // Serial.print(ary);   Serial.print("\t");
  // Serial.print(arz);   Serial.print("\t\t");
  // Serial.print(rx);   Serial.print("\t");
  // Serial.print(ry);   Serial.print("\t");
  // Serial.println(rz);

  i = i + 1;
  delay(2000);
}