/* Based on Edge Impulse Arduino examples
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * /

/* Includes ---------------------------------------------------------------- */

#include <Arduino.h>
#include <ML_IMU_inferencing.h>
#include <MKRIMU.h>
#include <cstdarg>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <utility/ECCX08SelfSignedCert.h>
#include <ArduinoMqttClient.h>
#include <MKRNB.h>
#include <SPI.h>
#include <SerialFlash.h>

#include "main.h"
#include "arduino_secrets.h"
#include "headCrypto.h"
#include "headIMU.h"

// Declaring Macros for SAMD21 Targets for Edge Impulse
#define __STATIC_FORCEINLINE                   __attribute__((always_inline)) static inline
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
void setup()
{
  Serial.begin(BAUD_RATE);
  
  setupECCX08();  

  // Set a callback to get the current time
  // used to validate the servers certificate
  ArduinoBearSSL.onGetTime(getTime);

  // Set the ECCX08 slot to use for the private key
  // and the accompanying public certificate for it
  sslClient.setEccSlot(0, ECCX08SelfSignedCert.bytes(), ECCX08SelfSignedCert.length());

  // Set the client id used for MQTT as the device id
  mqttClient.setId(deviceId);

  // Set the username to "<broker>/<device id>/api-version=2018-06-30" and empty password
  String username;

  username += broker;
  username += "/";
  username += deviceId;
  username += "/api-version=2018-06-30";

  mqttClient.setUsernamePassword(username, "");

  // Set the message callback, this function is
  // called when the MQTTClient receives a message
  mqttClient.onMessage(onMessageReceived);
    
  connectNB();
  connectMQTT();

  setupIMU();

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 9) 
  {
      Serial.println("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 9 (the 9 sensor axes)\n");
      return;
  }

  _currentTime = millis()/1000;
  _lastMessagePublished = _currentTime;

  initDataBuffer();
}

/**
 * @brief      Arduino main function
 */
void loop()
{

  float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

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
    delay(10);


  }
  reset_useBuffer();
  int use;
  for (size_t ix = 0; ix < 711; ix += 9)
  {
    if (abs(buffer[ix+15]-buffer[ix+6])<=0.05 && abs(buffer[ix+16]-buffer[ix+7])<=0.05 && abs(buffer[ix+17]-buffer[ix+8])<=0.05)
      _useBuffer[ix/9]=0;
    else
      _useBuffer[ix/9]=1;
  }

  if (is_useBufferEmpty())
  {
    if (_lastDetectedState != UNUSED)
    {
      endCurrentDataSegment();
      createNewDataSegment(UNUSED);
    }
    Serial.println("not used");
    use = 0;
    _lastDetectedState = UNUSED;
  }
  else
  {
    Serial.println("used");
    use = 1;
  }

  signal_t signal;
  int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (err != 0)
  {
    Serial.println("Failed to create signal from buffer" + err);
    return;
  }

  // Run the classifier
  ei_impulse_result_t result = { 0 };
  Serial.println("Running classifier...");
  err = run_classifier(&signal, &result, debug_nn);
  Serial.println("Classifier done running.");
  if (err != EI_IMPULSE_OK) 
  {
    Serial.println("ERR: Failed to run classifier " + err);
    return;
  }

  float largest_value = 0.f;
  int index_largest_value = 0;

  if (use == 1)
  {
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
    {
      if (static_cast<int>(result.classification[ix].value*100) > largest_value)
      {
        largest_value = static_cast<int>(result.classification[ix].value*100);
        index_largest_value = ix;
      }
    }

    if (static_cast<int>(result.anomaly*100) < MIN_SCORE_ANOMALY && largest_value > MIN_SCORE_CLASS)
    {
      //if largest score was the same for 3 classifications in a row, check if the last detected state is this one
      if (index_largest_value != _lastDetectedState)
      {
        //managing data
        Serial.println("Detected new state of use...\n");
        endCurrentDataSegment();
        createNewDataSegment(index_largest_value);
        _lastDetectedState = index_largest_value;
      }
    }
    else //anomaly detected
    {
      Serial.println("Detected anomaly...\n");
      if (_lastDetectedState != ANOMALY)
      {
        endCurrentDataSegment();
        createNewDataSegment(ANOMALY);
      }
      _lastDetectedState = ANOMALY;
      index_largest_value = ANOMALY;
    }
  }
  else
  {
    Serial.println("no movement\n");
  }

  // poll for new MQTT messages and send keep alives if still connected (useless otherwise)
  if (nbAccess.status() == NB_READY && gprs.status() == GPRS_READY &&
      mqttClient.connected())
    mqttClient.poll();

  _currentTime = millis()/1000;

  if (_currentTime - _lastMessagePublished > T_BETWEEN_MESSAGES)
  {
    if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) 
    {
      endCurrentDataSegment();
      createNewDataSegment(NOT_MEASURING);
      connectNB();
      endCurrentDataSegment();
    }

    if (!mqttClient.connected())
    {
      endCurrentDataSegment();
      createNewDataSegment(NOT_MEASURING);
      connectMQTT();
      endCurrentDataSegment();
    } 

    if (nbAccess.status() == NB_READY && gprs.status() == GPRS_READY &&
      mqttClient.connected())
    {
      endCurrentDataSegment();
      String message = makeMessageFromData();
      Serial.println(message.c_str());
      Serial.println("\n");
      publishMessage(message);
      clearDataBuffer();
      initDataBuffer();
    }
    /*Even if the board isn't connected, we set up the time of _lastMessagePublished
    so that it retries connecting in T_BETWEEN_MESSAGES, and still measures in between.*/
    _lastMessagePublished = _currentTime;
    _lastDetectedState = 20;
  }
}

