////////////////////////////////////////////////////////////////////////////  
 //  
 // This file is part of RTIMULib-Arduino  
 //  
 // Copyright (c) 2014-2015, richards-tech  
 //  
 // Permission is hereby granted, free of charge, to any person obtaining a copy of   
 // this software and associated documentation files (the "Software"), to deal in   
 // the Software without restriction, including without limitation the rights to use,   
 // copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the   
 // Software, and to permit persons to whom the Software is furnished to do so,   
 // subject to the following conditions:  
 //  
 // The above copyright notice and this permission notice shall be included in all   
 // copies or substantial portions of the Software.  
 //  
 // THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,   
 // INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A   
 // PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT   
 // HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION   
 // OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE   
 // SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.  
 //  
 // Further modified by King Tide Sailing (kingtidesailing.blogspot.com) 
 //  
 ////////////////////////////////////////////////////////////////////////////  
 #include "Wire.h"  
 #include "I2Cdev.h"  
 #include "RTIMUSettings.h"  
 #include "RTIMU.h"  
 #include "RTFusionRTQF.h"   
 #include "CalLib.h"  
 #include "EEPROM.h"  
 RTIMU *imu;                      // the IMU object  
 RTFusionRTQF fusion;                 // the fusion object  
 RTIMUSettings settings;                // the settings object  
 // DISPLAY_INTERVAL sets the rate at which results are displayed  
 #define DISPLAY_INTERVAL 100             // interval between pose displays  
 // SERIAL_PORT_SPEED defines the speed to use for the debug serial port  
 #define SERIAL_PORT_SPEED 115200  
 volatile unsigned long lastDisplay;  
 volatile unsigned long lastRate;  
 int sampleCount;  
 void setup()  
 {  
   int errcode;  
   Serial.begin(SERIAL_PORT_SPEED);  
   Wire.begin();  
   imu = RTIMU::createIMU(&settings);            // create the imu object  
   Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());  
   if ((errcode = imu->IMUInit()) < 0) {  
     Serial.print("Failed to init IMU: "); Serial.println(errcode);  
   }  
   if (imu->getCalibrationValid())  
     Serial.println("Using compass calibration");  
   else  
     Serial.println("No valid compass calibration data");  
   lastDisplay = lastRate = millis();  
   sampleCount = 0;  
   // Slerp power controls the fusion and can be between 0 and 1  
   // 0 means that only gyros are used, 1 means that only accels/compass are used  
   // In-between gives the fusion mix.  
   fusion.setSlerpPower(0.02);  
   // use of sensors in the fusion algorithm can be controlled here  
   // change any of these to false to disable that sensor  
   fusion.setGyroEnable(true);  
   fusion.setAccelEnable(true);  
   fusion.setCompassEnable(true);  
   Serial.println("Units");  
   Serial.println("Pose: Roll, Pitch, Magnetic Heading (measured in degrees)");  
   Serial.println("Gyro: Rate of Roll, Pitch, and Yaw (measured in degrees per second)");  
   Serial.println("Accl: Acceleration in X, Y, and Z (measured in g's, with 1 g equal to 9.8 meters per second squared)");  
   Serial.println("Time: Timestamp (measured in milliseconds from when the MPU-9150 was initiated)");  
   Serial.println("----------------------------");  
 }  
 void loop()  
 {   
   unsigned long now = millis();  
   unsigned long delta;  
   int loopCount = 1;  
   while (imu->IMURead()) {                // get the latest data if ready yet  
     // this flushes remaining data in case we are falling behind  
     if (++loopCount >= 10)  
       continue;  
     fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());  
     sampleCount++;  
     if ((delta = now - lastRate) >= 1000) {  
       sampleCount = 0;  
       lastRate = now;  
     }  
     if ((now - lastDisplay) >= DISPLAY_INTERVAL) {  
       lastDisplay = now;  
       RTVector3 pose = fusion.getFusionPose();  
       RTVector3 gyro = imu->getGyro();  
       RTVector3 accel = imu->getAccel();  
       float r = M_PI/180.0f;      // degrees to radians  
       float d = 180.0f/M_PI;      // radians to degrees  
       float roll = pose.y()*d*-1;    // left roll is negative  
       float pitch = pose.x()*d;     // nose down is negative  
       float yaw = pose.z()*d;      // 0 Yaw = 270 magnetic, this gives left or right up to 180 degrees  
       float hdm = yaw-90;        // converts yaw to heading magnetic  
       if (yaw < 90 && yaw >= -179.99) {  
        hdm = yaw+270;  
       }  
       float ror = gyro.y()*-1*d;    // rate of roll  
       float rop = gyro.x()*d;      // rate of pitch  
       float rot = gyro.z()*d;      // rate of turn  
       float ax = accel.x();       // acceleration on the x-axis  
       float ay = accel.y();       // acceleration on the y-axis  
       float az = accel.z();       // acceleration on the z-axis  
       volatile int ts = imu->getTimestamp();   // the timestamp  
       Serial.print("Pose: ");  
       Serial.print(roll);  
       Serial.print(" | ");  
       Serial.print(pitch);  
       Serial.print(" | ");  
       Serial.println(hdm);  
       Serial.print("Gyro: ");  
       Serial.print(ror);  
       Serial.print(" | ");  
       Serial.print(rop);  
       Serial.print(" | ");  
       Serial.println(rot);  
       Serial.print("Accl: ");  
       Serial.print(ax);  
       Serial.print(" | ");  
       Serial.print(ay);  
       Serial.print(" | ");  
       Serial.println(az);  
       Serial.print("Time: ");  
       Serial.println(ts);  
       Serial.println("----------------------------");  
     }  
   }  
 }  