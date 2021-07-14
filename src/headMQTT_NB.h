/*
Header file for MQTT protocol  
 */

#ifndef HEAD_MQTT_NB_H_
#define HEAD_MQTT_NB_H_

#include <ArduinoMqttClient.h>
#include "arduino_secrets.h"

#include <MKRNB.h>

MqttClient *head_mqttClient;
NB *head_nbAccess;
GPRS *head_gprs;

// Enter your sensitive data in arduino_secrets.h
const char pinnumber[]   = SECRET_PINNUMBER;
const char broker[]      = SECRET_BROKER;
String     deviceId      = SECRET_DEVICE_ID;

/**
 * @brief Connects the board to the cellular network. 
 */
void connectNB(NB *inpNB, GPRS *inpGprs);

/**
 * @brief Connects the board using the MQTT protocol.
 * 
 * This connection allows us to publish messages in the IoT hub.
 */
void connectMQTT();

/**
 * @brief Publish a message on the IoT hub.
 * 
 * @param message Message to be sent.
 */
void publishMessage(String message);

/**
 * @brief Prints a message received from the IoT hub on the serial monitor.
 * 
 * @param messageSize Size of the message, as an integer.
 */
void onMessageReceived(int messageSize);

void setupMQTT_NB(MqttClient *inp_mqttClient, NB *inpNB, GPRS *inpGprs);


void publishMessage(String message) {
  Serial.println("Publishing message");
  head_mqttClient->beginMessage("devices/" + deviceId + "/messages/events/");
  head_mqttClient->print(message);
  head_mqttClient->endMessage();
}

void connectNB(NB *inpNB, GPRS *inpGprs) {
  head_nbAccess = inpNB;  
  head_gprs = inpGprs;
  Serial.println("Attempting to connect to the cellular network");

  unsigned long time_beginning = millis()/1000;
  unsigned long current_time = time_beginning;
  
  while (((head_nbAccess->begin(pinnumber) != NB_READY) ||
         (head_gprs->attachGPRS() != GPRS_READY)) &&
         current_time - time_beginning >= 30) {
    // failed, retry for 30 seconds
    Serial.print(".");
    delay(500);
    current_time = millis()/1000;
  }

  Serial.println("You're connected to the cellular network");
  Serial.println();
}

void connectMQTT() {
  Serial.print("Attempting to MQTT broker: ");
  Serial.print(broker);
  Serial.println(" ");

  unsigned long time_beginning = millis()/1000;
  unsigned long current_time = time_beginning;

  while (!head_mqttClient->connect(broker, 8883) &&
      current_time - time_beginning >= 30) {
    // failed, retry for 30 seconds
    Serial.print(".");
    Serial.println(head_mqttClient->connectError());
    delay(5000);
    current_time = millis()/1000;
  }
  Serial.println();

  Serial.println("You're connected to the MQTT broker");
  Serial.println();

  // subscribe to a topic
  head_mqttClient->subscribe("devices/" + deviceId + "/messages/devicebound/#");

  publishMessage("Board online.");
}

void onMessageReceived(int messageSize) {
  // we received a message, print out the topic and contents
  Serial.print("Received a message with topic '");
  Serial.print(head_mqttClient->messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (head_mqttClient->available()) 
    Serial.print((char)head_mqttClient->read());
   
  Serial.println();
}


void setupMQTT_NB(MqttClient *inp_mqttClient, NB *inpNB, GPRS *inpGprs){
    head_mqttClient = inp_mqttClient;
    // Set the client id used for MQTT as the device id
    head_mqttClient->setId(deviceId);

    // Set the username to "<broker>/<device id>/api-version=2018-06-30" and empty password
    String username;

    username += broker;
    username += "/";
    username += deviceId;
    username += "/api-version=2018-06-30";

    head_mqttClient->setUsernamePassword(username, "");

    // Set the message callback, this function is
    // called when the MQTTClient receives a message
    head_mqttClient->onMessage(onMessageReceived);
        
    connectNB(inpNB, inpGprs);
    connectMQTT();
}

#endif