/**
 * Master code that reads IMU data and performs NN
 */
#include <Arduino.h>
#include <ML_IMU_inferencing.h>
#include <cstdarg>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <utility/ECCX08SelfSignedCert.h>
#include <ArduinoMqttClient.h>
#include <SPI.h>
#include <SerialFlash.h>

#include "main.h"
#include "arduino_secrets.h"
#include "headCrypto.h"
#include "headIMU.h"
// #include "headMQTT_NB.h"

const int slavePin = 4;

// Declaring Macros for SAMD21 Targets for Edge Impulse
#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE                   __attribute__((always_inline)) static inline
#endif
#define __SSAT(ARG1, ARG2) \
    __extension__ \
    ({                          \
      int32_t __RES, __ARG1 = (ARG1); \
      __ASM volatile ("ssat %0, %1, %2" : "=r" (__RES) :  "I" (ARG2), "r" (__ARG1) : "cc" ); \
      __RES; \
     })

/**
 * @brief      Arduino setup function
 */
void setup(){
  Serial.begin(BAUD_RATE);
  
  setupECCX08();  
  
  setupIMU();

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 9) 
  {
      Serial.println("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 9 (the 9 sensor axes)\n");
      return;
  }

  _currentTime = millis()/1000;
  _lastMessagePublished = _currentTime;

  initDataBuffer();

// Slave Select
pinMode(slavePin, OUTPUT);
digitalWrite(slavePin, HIGH);

// Initialise and Setup SPI
SPI.begin();
SPI.setClockDivider(SPI_CLOCK_DIV4);
SPI.setBitOrder(MSBFIRST);
SPI.setDataMode(SPI_MODE0);
}

void loop(){
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
  
    float *dataBuffer = readIMU(buffer);

    reset_useBuffer();
    int use;
    for (size_t ix = 0; ix < 9 * (WINDOW_SIZE - 1); ix += 9){
        if (abs(dataBuffer[ix+15] - dataBuffer[ix+6])<=0.05 && abs(buffer[ix+16]-buffer[ix+7])<=0.05 && abs(buffer[ix+17]-buffer[ix+8])<=0.05)
        _useBuffer[ix/9]=0;
        else
        _useBuffer[ix/9]=1;
    }

    if (is_useBufferEmpty()){
        if (_lastDetectedState != UNUSED){
        endCurrentDataSegment();
        createNewDataSegment(UNUSED);
        }
        // Serial.println("not used");
        use = 0;
        _lastDetectedState = UNUSED;
    }
    else{
        Serial.println("used");
        use = 1;
    }

    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0){
        // Serial.println("Failed to create signal from buffer" + err);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };
    Serial.println("Running classifier...");
    err = run_classifier(&signal, &result, debug_nn);
    Serial.println("Classifier done running.");
    err = EI_IMPULSE_OK;
    if (err != EI_IMPULSE_OK) {
        Serial.println("ERR: Failed to run classifier " + err);
        return;
    }

    float largest_value = 0.f;
    int index_largest_value = 0;

    if (use == 1){
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++){
            if (static_cast<int>(result.classification[ix].value*100) > largest_value){
                largest_value = static_cast<int>(result.classification[ix].value*100);
                index_largest_value = ix;
            }
        }

        if (static_cast<int>(result.anomaly*100) < MIN_SCORE_ANOMALY && largest_value > MIN_SCORE_CLASS){
        //if largest score was the same for 3 classifications in a row, check if the last detected state is this one
            if (index_largest_value != _lastDetectedState){
                //managing data
                // Serial.println("Detected new state of use...\n");
                endCurrentDataSegment();
                createNewDataSegment(index_largest_value);
                _lastDetectedState = index_largest_value;
            }
        }
        // anomaly detected
        else{
        // Serial.println("Detected anomaly...\n");
            if (_lastDetectedState != ANOMALY){
                endCurrentDataSegment();
                createNewDataSegment(ANOMALY);
            }
        _lastDetectedState = ANOMALY;
        index_largest_value = ANOMALY;
        }
    }
    else{
        Serial.println("no movement\n");
    }
    // https://stackoverflow.com/questions/505021/get-bytes-from-stdstring-in-c
    
    // Send to Slave MKR
    digitalWrite(slavePin, LOW);
    // SPI.beginTransaction(DEFAULT_SPI_SETTINGS);
    // SPI.transfer(42);
    SPI.transfer(getIntState());
    // SPI.endTransaction();
    Serial.println("Data Transferred Over SPI");
    digitalWrite(slavePin, HIGH);
}