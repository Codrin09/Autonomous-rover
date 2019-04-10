/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

#include "MPU6050.h"
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az, gx, gy, gz;

double timeStep;
volatile double timeGyro,timePrev;
double arx, ary, arz, grx, gry, grz, gsx, gsy, gsz, rx, ry, rz;

double gyroScale = 131;

void setup_gyro() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    accelgyro.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(accelgyro.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    timeGyro = millis();
}

//Update gyroscope with last readings from FIFO
float get_gyro(bool initGyro){
    // set up time for integration
    timePrev = timeGyro;
    timeGyro = millis();
    timeStep = (timeGyro - timePrev) / 1000; // time-step in s

    // collect readings
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // apply gyro scale from datasheet
    gsx = gx/gyroScale;   gsy = gy/gyroScale;   gsz = gz/gyroScale;

    // calculate accelerometer angles
    arx = (180/M_PI) * atan(ax / sqrt(pow(ay, 2) + pow(az, 2))); 
    ary = (180/M_PI) * atan(ay / sqrt(pow(ax, 2) + pow(az, 2)));
    arz = (180/M_PI) * atan(sqrt(pow(ay, 2) + pow(ax, 2)) / az);

    // set initial values equal to accel values
    if (initGyro) {
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

    return grx;
}