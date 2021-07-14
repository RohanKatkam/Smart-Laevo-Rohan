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

float * readIMU(float buffer[]){
    

    // WINDOW SIZE IS 80
    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 9) 
    { 
        //EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE = 720
        // Determine the next tick (and then sleep later)
        //uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
        float accx, accy, accz;
        float gyrx, gyry, gyrz;
        float heading, roll, pitch;

        // Reading accelerometer data from sensor and append in buffer
        IMU.readAcceleration(accx, accy, accz);
        buffer[ix + 0] = accx;
        buffer[ix + 1] = accy;
        buffer[ix + 2] = accz;

        // Reading gyroscope data from sensor and append in buffer
        IMU.readGyroscope(gyrx, gyry, gyrz);
        buffer[ix + 3] = gyrx;
        buffer[ix + 4] = gyry;
        buffer[ix + 5] = gyrz;

        // Reading orientation data from sensor and append in buffer
        IMU.readEulerAngles(heading, roll, pitch);
        buffer[ix + 6] = heading;
        buffer[ix + 7] = roll;
        buffer[ix + 8] = pitch;
        
        // reads every 10 ms, put an interrupt instead
        delay(10);
    }

    return buffer;
}


#endif