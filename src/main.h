//-----------------------------------------------------------------------------------------------
#ifndef MAIN_H
#define MAIN_H

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

#include "arduino_secrets.h"

#define BAUD_RATE 115200

/**
 * @brief Gets the current time from the cellular module.
 * 
 * @returns Current time according to the cellular module.
 * @note Do not get mixed up with the millis() method (also used in this program
 * for setting the _currentTime and _lastMessagePublished values) that returns
 * time relative to the beginning of the currently running program. 
 */
unsigned long getTime();

/**
* @brief Initialize a new data buffer.
*
* Clears the data buffer in case it is not done, and resets the counter.
*/
void initDataBuffer();

/**
 * @brief Sets the end of the buffer flag for the data structure.
 */
void setEndOfDataBuffer();

/**
* @brief Adds a new data segment to the data buffer.
*/
void createNewDataSegment(int stateFlag);

/**
* @brief Change the values of the current data segment to their ending values.
*/
void endCurrentDataSegment();

/**
* @brief Clears the data buffer.
*/
void clearDataBuffer();

/**
* @brief Makes a message using the current values in data.
*
* @returns A String corresponding to the message to send afterwards.
* It has the following format : STATE DURATION T_BEGIN T_END |
* @note It is intended to be called right before the sending of the said message.
* The pattern should go as follow : after a given time or signal, make the message,
* broadcast it, reset the data and start again.
*/
String makeMessageFromData();

/**
* @brief      Sets all values of _useBuffer to 0.
*/
void reset_useBuffer();

/**
* @brief         Checks if the values of _useBuffer are all equal to 0.
*
* @returns        true if the buffer values are all equal to 0, false otherwise.
*/
bool is_useBufferEmpty();


// Constants:
#define WINDOW_SIZE         80
#define BUF_SIZE            256

/*This constant is used as an emergency flag for the browsing of _data.
It should always be placed at _data[_dataCount][0].
It can be changed to virtually any value, except those used for the use
identification, as it is placed at the same index as the use flags.*/
#define END_OF_BUFFER       112624

// Those must be the same as in EI API
#define MIN_SCORE_ANOMALY   30.0f // min anomaly score to label as anomaly as defined in Edge Impulse
#define MIN_SCORE_CLASS     80.0f // min classification score to label as anomaly as defined in Edge Impulse

#define T_BETWEEN_MESSAGES  5 //in seconds

// Use flags
#define STOOPING            1
#define WALKING             2
#define SQUATTING           3
#define STANDING            4
#define UNUSED              5
#define ANOMALY             6
//used when the micro-controller is reconnecting
#define NOT_MEASURING       7

// NB nbAccess;
// GPRS gprs;

// NBClient      nbClient;            // Used for the TCP socket connection
// BearSSLClient sslClient(nbClient); // Used for SSL/TLS connection, integrates with ECC508
// MqttClient    mqttClient(sslClient);

unsigned long _currentTime; //converted to seconds in the code
unsigned long _lastMessagePublished; //converted to seconds in the code

int _lastDetectedState = 20; //the state/label that was lastly counted three times succesfully

static bool debug_nn = true; // Set this to true to see e.g. features generated from the raw signal

const size_t kusebufferlen = 79;
float _useBuffer[kusebufferlen]; //buffer for detecting the use of the exoskeleton
float _zeroUseBuffer[kusebufferlen];
/*Data buffer, storing the data segments.
The stored data takes the following form :
index 0 : state tag;
index 1 : duration of the state;
index 2 : the time at which the state was entered;
index 3 : the time at which the state was exited.
Example : data[2][1] is the total duration of the third data segment.
*/
int _data[BUF_SIZE][4];

/*should be equal to the column count of the data buffer, i.e. the total
amount of data segments. Note that in most languages, arrays start
at 0, meaning the last data segment in the data buffer is located
at data[_dataCount-1].*/
size_t _dataCount;


void setEndOfDataBuffer()
{
  _data[_dataCount][0] = END_OF_BUFFER; //set end of buffer flag
}

void clearDataBuffer()
{
  Serial.println("Data being deleted...\n");
  for (int i = _dataCount; i >= 0; --i)
  {
    _data[i][0] = -1;
    _data[i][1] = -1;
    _data[i][2] = -1;
    _data[i][3] = -1;
  }
  Serial.println("Data deleted.\n");
}

void initDataBuffer()
{
  clearDataBuffer(); //safety check
  _dataCount = 0;
  setEndOfDataBuffer();
}

void createNewDataSegment(int stateFlag)
{
  if (_dataCount == BUF_SIZE-1)
  {
    Serial.println("Maximum buffer size reached : data lost.");
    return;
  }
  Serial.println("Creating new data...\n");
  _currentTime = millis()/1000;

  //adding the new data.
  _data[_dataCount][0] = stateFlag; //state of use.
  _data[_dataCount][1] = _currentTime; //total duration
  _data[_dataCount][2] = _currentTime; //time beginning
  _data[_dataCount][3] = stateFlag;; //time ending

  _dataCount++;
  
  Serial.println("_dataCount : ");
  Serial.println(String(_dataCount).c_str());
  Serial.println("\n");

  setEndOfDataBuffer();

  Serial.println("Data created and added successfully.\n");
}

void endCurrentDataSegment()
{
  Serial.println("Ending current data.\n");
  if (_dataCount!=0) //case check if it is the first new data we're recording.
  {
    Serial.println("Not first data; getting the right time...\n");
    _currentTime = millis()/1000;

    //setting the duration of the use, and the last use time 
    _data[_dataCount-1][1] = _currentTime - _data[_dataCount-1][1];
    _data[_dataCount-1][3] = _currentTime;
  }
  Serial.println("Data ended successfully.\n");
}

String makeMessageFromData()
{
  /*It might be worth considering to implement a way to make "continuous" data;
  for instance, if we send a message that ends with a certain state, most of the
  time the next message will begin with the same state in data, but might lead
  to discontinuities when processing the results in the end. It might bee a
  back-end problem, or it could maybe be solved here.*/
  Serial.println("Instanciating message...\n");
  String message = "";
  Serial.println("Message instanciated.\nScanning data for completing message...\n");
  for (size_t i = 0; i < _dataCount; i++)
  {
    switch (_data[i][0])
    {
      case STOOPING :
        message += "Stooping ";
        break;

      case WALKING :
        message += "Walking ";
        break;

      case SQUATTING :
        message += "Squatting ";
        break;

      case STANDING :
        message += "Standing ";
        break;

      case UNUSED :
        message += "Unused ";
        break;

      case ANOMALY :
        message += "Anomaly ";
        break;

      case NOT_MEASURING :
        message += "Not measuring ";
        break;

      case END_OF_BUFFER :
        /*Safety check.*/
        Serial.println("Finished browsing data.\n");
        return message;

      default :
        message += "Error |";
        return message;
    }
    //total duration of the actual mode.
    message += String(_data[i][1]);
    message += " ";
    //beginning time of the use of this mode.
    message += String(_data[i][2]);
    message += " ";
    //ending time of the use of this mode.
    message += String(_data[i][3]);
    message += " |";
  }
  return message; 
}

void reset_useBuffer()
{
  // for (size_t i = 0; i < kusebufferlen; i++)
  //   _useBuffer[i] = 0.f;
  memset(_useBuffer, 0.f, sizeof(_useBuffer));
}

bool is_useBufferEmpty(){
  // https://stackoverflow.com/questions/12866413/comparing-arrays-for-equality-in-c
  return (memcmp(_zeroUseBuffer, _useBuffer, sizeof(_useBuffer)) == 0);
}

unsigned long getTime() 
{
  // return nbAccess.getTime();
  return 2.5; // some random number
}

#endif