/* 
* Header file for IMU 
*/

#ifndef HEAD_IMU_H_
#define HEAD_IMU_H_

#include <MKRIMU.h>

void setupIMU(){
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");

        //if we can't connect to the IMU, we retry every second
        while (1){
                if (IMU.begin())
                    break;
                
                delay(1000);
        }
    }
}

#endif