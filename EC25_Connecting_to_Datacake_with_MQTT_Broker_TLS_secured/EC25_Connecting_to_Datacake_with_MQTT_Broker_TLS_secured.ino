#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>  // Include the WiFi library for MAC address
#include <ArduinoJson.h>
#include "Secret.h" // Include the file to get the username and password of MQTT server
#include"datacake.h"

String gsm_send_serial(String command, int delay);

//#define TINY_GSM_MODEM_SIM7600
#define SerialMon Serial
#define SerialAT Serial1
#define GSM_PIN ""

// Your GPRS credentials
const char apn[] = "dialogbb";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT details
String broker = "mqtt.datacake.co";
String MQTTport = "8883";

#define uS_TO_S_FACTOR 1000000ULL // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP 60          // Time ESP32 will go to sleep (in seconds)

#define UART_BAUD 115200

#define MODEM_TX 32
#define MODEM_RX 33
#define GSM_RESET 21
#define D0 39
#define D1 34
#define D2 35
#define D3 14
#define R0 12

#define MAC_ADDRESS_SIZE 18 // Assuming MAC address is in format "XX:XX:XX:XX:XX:XX"
byte mac[6];
String str_macAddress;

unsigned long prevMillis = 0;
const unsigned long interval = 60000; // Interval for sending messages

// Device-specific details
const char* deviceSerial = "3CE90E6C8F88";  // Replace with your device serial

void mqttCallback(char* topic, String payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
//  SerialMon.write(payload, len);
//  SerialMon.println();

  // Extract serial number from the topic
  String topicStr = String(topic);
  int firstSlash = topicStr.indexOf('/');
  int lastSlash = topicStr.lastIndexOf('/');
  String MAC_ID = topicStr.substring(firstSlash + 1, lastSlash);

  SerialMon.print("MAC ID: ");
  SerialMon.println(MAC_ID);

    // Extract the payload
    int state = payload.toInt();
    SerialMon.print("STATE: ");
     SerialMon.println(state);

    // Handle state changes here
     if (state == 0) {
       digitalWrite(R0, LOW);
     } else if (state == 1) {
       digitalWrite(R0, HIGH);
     }

}

void setup() {
  // Set console baud rate
  Serial.begin(115200);
  delay(10);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);

  delay(2000);
  pinMode(GSM_RESET, OUTPUT);
  digitalWrite(GSM_RESET, HIGH);  
  delay(2000);
   pinMode(D0, INPUT);
   pinMode(D1, INPUT);
   pinMode(D2, INPUT);
   pinMode(D3, INPUT);
   pinMode(R0, OUTPUT);

  Init();
  connectToGPRS();
  connectToMQTT();
}

void loop() {
  if (millis() - prevMillis >= interval) {
    prevMillis = millis();

    // Read input values (assuming these are commented out for now)
     bool IN1 = digitalRead(D0);
     bool IN2 = digitalRead(D1);
     bool IN3 = digitalRead(D2);
     bool IN4 = digitalRead(D3);

    String DI0 = String(IN1 ? 1 : 0);
    String DI1 = String(IN2 ? 1 : 0);
    String DI2 = String(IN3 ? 1 : 0);
    String DI3 = String(IN4 ? 1 : 0);

    String Digital_input_0 = "dtck-pub/dc_mqtt_broker/9ccc450d-96ec-4676-8e57-a3661bf528a6/D0";
    String Digital_input_1 = "dtck-pub/dc_mqtt_broker/9ccc450d-96ec-4676-8e57-a3661bf528a6/D1";
    String Digital_input_2 = "dtck-pub/dc_mqtt_broker/9ccc450d-96ec-4676-8e57-a3661bf528a6/D2";
    String Digital_input_3 = "dtck-pub/dc_mqtt_broker/9ccc450d-96ec-4676-8e57-a3661bf528a6/D3";

    String command_0 = "AT+QMTPUBEX=0,1,1,0,\"" + Digital_input_0 + "\"," +String(DI0.length());
    String publishResponse_0 = gsm_send_serial(command_0, 1000);
    publishResponse_0 += gsm_send_serial(DI0 + "\x1A", 1000);
    
    String command_1 = "AT+QMTPUBEX=0,1,1,0,\"" + Digital_input_1 + "\"," +String(DI1.length());
    String publishResponse_1 = gsm_send_serial(command_1, 1000);
    publishResponse_1 += gsm_send_serial(DI1 + "\x1A", 1000);
    
    String command_2 = "AT+QMTPUBEX=0,1,1,0,\"" + Digital_input_2 + "\"," +String(DI2.length());
    String publishResponse_2 = gsm_send_serial(command_2, 1000);
    publishResponse_2 += gsm_send_serial(DI2 + "\x1A", 1000);
    
    String command_3 = "AT+QMTPUBEX=0,1,1,0,\"" + Digital_input_3 + "\"," +String(DI3.length());
    String publishResponse_3 = gsm_send_serial(command_3, 1000);
    publishResponse_3 += gsm_send_serial(DI3 + "\x1A", 1000);

    // Check if the publish command was successful
    if (publishResponse_0.indexOf("ERROR") != -1) {
      SerialMon.println("MQTT publish failed. Reconnecting...");
      connectToMQTT();
      if (!isGPRSConnected()) {
        SerialMon.println("GPRS connection lost. Reconnecting...");
        connectToGPRS();
        connectToMQTT();
      }
      if (!isNetworkConnected()) {
        SerialMon.println("Network connection lost. Reconnecting...");
        Init();
        connectToGPRS();
        connectToMQTT();
      }
    }
  }
  // Handle incoming MQTT messages
  handleIncomingMessages();
}

void handleIncomingMessages() {
  // Request messages from the EC25 module
  String response = gsm_send_serial("AT+QMTRECV=0,1", 1000);

  // Print the raw response for debugging
  SerialMon.print("Raw MQTT Response: ");
  SerialMon.println(response);

  // Check if the response contains "+QMTRECV:"
  int startPos = response.indexOf("+QMTRECV:");
  if (startPos != -1) {
    // Extract the part of the response containing the message
    String messagePart = response.substring(startPos);

    // Print the extracted message part for debugging
    SerialMon.print("Extracted Message Part: ");
    SerialMon.println(messagePart);

    // Remove any extraneous text before "+QMTRECV:"
    messagePart.trim();
    
    // Check if the response is in the expected format
    if (messagePart.startsWith("+QMTRECV:")) {
      // Extract the part after "+QMTRECV:" (skip the "+QMTRECV:" prefix)
      messagePart = messagePart.substring(messagePart.indexOf(':') + 1);
      
      // Extract client_idx and msg_id
      int firstComma = messagePart.indexOf(',');
      int secondComma = messagePart.indexOf(',', firstComma + 1);
      String client_idx = messagePart.substring(0, firstComma);
      String msg_id = messagePart.substring(firstComma + 1, secondComma);

      // Extract topic
      int firstQuote = messagePart.indexOf('"', secondComma + 1);
      int secondQuote = messagePart.indexOf('"', firstQuote + 1);
      String topic = messagePart.substring(firstQuote + 1, secondQuote);

      // Extract payload length
      int thirdComma = messagePart.indexOf(',', secondQuote + 1);
      int fourthComma = messagePart.indexOf(',', thirdComma + 1);
      String payloadLengthStr = messagePart.substring(thirdComma + 1, fourthComma);
      int payloadLength = payloadLengthStr.toInt();

      // Extract payload
      int thirdQuote = messagePart.indexOf('"', fourthComma + 1);
      int fourthQuote = messagePart.indexOf('"', thirdQuote + 1);
      String payload = messagePart.substring(thirdQuote + 1,  fourthQuote );

      // Debug print
      SerialMon.print("Received Topic: ");
      SerialMon.println(topic);
      SerialMon.print("Received Payload: ");
      SerialMon.println(payload);
      SerialMon.print("Payload Length: ");
      SerialMon.println(payloadLength);

      // Convert topic and payload to mutable char arrays
      char topicArr[topic.length() + 1];
     // byte payloadArr[payload.length() + 1];

      topic.toCharArray(topicArr, topic.length() + 1);
      
      // Call the MQTT callback function with the extracted values
      mqttCallback(topicArr, payload, payload.length());
    } else {
      SerialMon.println("Unexpected response format.");
    }
  } else {
    SerialMon.println("No new MQTT messages or unexpected response format.");
  }
}

void Init(void) {                        // Connecting with the network and GPRS
  delay(5000);
  gsm_send_serial("AT+CFUN=1", 10000);
  gsm_send_serial("AT+CPIN?", 10000);
  gsm_send_serial("AT+CSQ", 1000);
  gsm_send_serial("AT+CREG?", 1000);
  gsm_send_serial("AT+COPS?", 1000);
  gsm_send_serial("AT+CGATT?", 1000);
  gsm_send_serial("AT+CPSI?", 500);
  gsm_send_serial("AT+CGDCONT=1,\"IP\",\"dialogbb\"", 1000);
  gsm_send_serial("AT+CGACT=1,1", 1000);
  gsm_send_serial("AT+CGATT?", 1000);
  gsm_send_serial("AT+CGPADDR=1", 500);
}

void connectToGPRS(void) {
  gsm_send_serial("AT+CGATT=1", 1000);
  gsm_send_serial("AT+CGDCONT=1,\"IP\",\"dialogbb\"", 1000);
  gsm_send_serial("AT+CGACT=1,1", 1000);
  gsm_send_serial("AT+CGPADDR=1", 500);
}

void connectToMQTT(void) {
  // Initialize MQTT configurations
  gsm_send_serial("AT+QMTCFG=\"recv/mode\",0,0,1", 1000);
  gsm_send_serial("AT+QMTCFG=\"SSL\",0,1,2", 1000);
  int cert_length = mqtt_ca_cert.length(); // Get the length of the CA certificate
  String ca_cert = "AT+QFUPL=\"RAM:datacake_ca.pem\"," + String(cert_length) + ",100";
  gsm_send_serial(ca_cert, 1000); // Send the command
  delay(1000);
  gsm_send_serial(mqtt_ca_cert, 1000); // Send the command to upload CA singned certificate
  delay(1000);
  gsm_send_serial("AT+QSSLCFG=\"cacert\",2,\"RAM:datacake_ca.pem\"", 1000);
  gsm_send_serial("AT+QMTOPEN=0,\"mqtt.datacake.co\",8883", 1000);
  delay(2000); // Wait for the connection to establish
  String mqtt_conn = "AT+QMTCONN=0,\"EC25 client\",\"" + username + "\",\"" + password + "\"";
  gsm_send_serial(mqtt_conn, 1000);
  delay(2000); // Wait for the connection to establish

  // Subscribe to the downlink topic
  String downlinkTopic = "dtck/dc_mqtt_broker/9ccc450d-96ec-4676-8e57-a3661bf528a6/R1";
  String subscribeCommand = "AT+QMTSUB=0,1,\"" + downlinkTopic + "\",0"; // QoS 1
  gsm_send_serial(subscribeCommand, 1000);
  delay(2000); // Allow time for subscription confirmation

  // Check for subscription confirmation
  String response = gsm_send_serial("AT+QMTSUB?", 1000); // Check subscription status
  SerialMon.print("Subscription Response: ");
  SerialMon.println(response);

  // Debug: Print MQTT connection status
  String connStatus = gsm_send_serial("AT+QMTCONN?", 1000);
  SerialMon.print("MQTT Connection Status: ");
  SerialMon.println(connStatus);
}

bool isNetworkConnected() {
  String response = gsm_send_serial("AT+CREG?", 3000);
  return (response.indexOf("+CREG: 0,1") != -1 || response.indexOf("+CREG: 0,5") != -1);
}

bool isGPRSConnected() {
  String response = gsm_send_serial("AT+CGATT?", 3000);
  return (response.indexOf("+CGATT: 1") != -1);
}

String gsm_send_serial(String command, int timeout) {
  String buff_resp = "";
  Serial.println("Send ->: " + command);
  SerialAT.println(command);
  unsigned long startMillis = millis();
  
  while (millis() - startMillis < timeout) {
    while (SerialAT.available()) {
      char c = SerialAT.read();
      buff_resp += c;
    }
    delay(10); // Small delay to allow for incoming data to accumulate
  }
  
  Serial.println("Response ->: " + buff_resp);
  return buff_resp;
}
