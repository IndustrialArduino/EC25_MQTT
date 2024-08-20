#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>  // Include the WiFi library for MAC address
#include <ArduinoJson.h>
#include "Secret.h" // Include the file to get the username and password of MQTT server

String gsm_send_serial(String command, int delay);

//#define TINY_GSM_MODEM_SIM7600
#define SerialMon Serial
#define SerialAT Serial1
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "dialogbb";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT details
String broker = "mqtt2.sensoper.net";
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

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();

  // Extract serial number from the topic
  String topicStr = String(topic);
  int firstSlash = topicStr.indexOf('/');
  int lastSlash = topicStr.lastIndexOf('/');
  String MAC_ID = topicStr.substring(firstSlash + 1, lastSlash);

  SerialMon.print("MAC ID: ");
  SerialMon.println(MAC_ID);

  if (MAC_ID == deviceSerial) {
    // Decode the received message
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload, len);

    if (error) {
      SerialMon.print("deserializeJson() failed: ");
      SerialMon.println(error.c_str());
      return;
    }

    // Extract the payload
    bool state = doc["state"];
    SerialMon.print("STATE: ");
     SerialMon.println(state);

    // Handle state changes here
     if (state == 0) {
       digitalWrite(R0, LOW);
     } else if (state == 1) {
       digitalWrite(R0, HIGH);
     }
  } else {
    SerialMon.println("Received message for a different serial number");
  }
}

void setup() {
  // Set console baud rate
  Serial.begin(115200);
  delay(10);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);

  delay(2000);
  pinMode(GSM_RESET, OUTPUT);
  digitalWrite(GSM_RESET, HIGH);  // RS-485
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


    // Create JSON object
    StaticJsonDocument<200> doc;
     doc["D0"] = IN1 ? 1 : 0;
     doc["D1"] = IN2 ? 1 : 0;
     doc["D2"] = IN3 ? 1 : 0;
     doc["D3"] = IN4 ? 1 : 0;

    String jsonString;
    serializeJson(doc, jsonString);

    WiFi.macAddress(mac);

    str_macAddress = (String(mac[0] >> 4, HEX) + String(mac[0] & 0x0F, HEX)) +
                     (String(mac[1] >> 4, HEX) + String(mac[1] & 0x0F, HEX)) +
                     (String(mac[2] >> 4, HEX) + String(mac[2] & 0x0F, HEX)) +
                     (String(mac[3] >> 4, HEX) + String(mac[3] & 0x0F, HEX)) +
                     (String(mac[4] >> 4, HEX) + String(mac[4] & 0x0F, HEX)) +
                     (String(mac[5] >> 4, HEX) + String(mac[5] & 0x0F, HEX));
    str_macAddress.toUpperCase();
    String Digital_input = "NORVI/INPUTS/" + str_macAddress;
    
    SerialMon.print("Published: ");
    SerialMon.println(jsonString);

    // Publish MQTT message and check the result
    String publishResponse = publishMQTTMessage(Digital_input,jsonString, 1, 0);  // QoS 1, retain 0

    // Check if the publish command was successful
    if (publishResponse.indexOf("ERROR") != -1) {
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

String publishMQTTMessage(String topic, String message, int qos, int retain) {
  // Convert topic and message to be used with AT+QMTPUBEX
  int topicLength = topic.length();
  int messageLength = message.length();

  // Publish command
  String command = "AT+QMTPUBEX=0,1," + String(qos) + "," + String(retain) + ",\"" + topic + "\"," + String(messageLength);
  String response = gsm_send_serial(command, 1000);

  // Send the actual payload
  response += gsm_send_serial(message + "\x1A", 1000);  // Append end-of-message character if needed

  // Print response for debugging
  SerialMon.print("Publish Response: ");
  SerialMon.println(response);

  return response; // Return response for further handling
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
      int fourthQuote = messagePart.indexOf('}', thirdQuote + 1);
      int fifthQuote = messagePart.indexOf('"', fourthQuote + 1);
      String payload = messagePart.substring(thirdQuote + 1,  fifthQuote );

      // Debug print
      SerialMon.print("Received Topic: ");
      SerialMon.println(topic);
      SerialMon.print("Received Payload: ");
      SerialMon.println(payload);
      SerialMon.print("Payload Length: ");
      SerialMon.println(payloadLength);

      // Convert topic and payload to mutable char arrays
      char topicArr[topic.length() + 1];
      byte payloadArr[payload.length() + 1];

      topic.toCharArray(topicArr, topic.length() + 1);
      for (int i = 0; i < payload.length(); ++i) {
        payloadArr[i] = (byte)payload[i];
      }
      payloadArr[payload.length()] = '\0'; // Null-terminate byte array

      // Call the MQTT callback function with the extracted values
      mqttCallback(topicArr, payloadArr, payload.length());
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
  gsm_send_serial("AT+QMTOPEN=0,\"mqtt2.sensoper.net\",8883", 1000);
  delay(2000); // Wait for the connection to establish
  String command = "AT+QMTCONN=0,\"EC25 client\",\"" + username + "\",\"" + password + "\"" ;
  gsm_send_serial(command, 1000);
  delay(2000); // Wait for the connection to establish

  // Subscribe to the downlink topic
  String downlinkTopic = "NORVI/+/OUTPUT";
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
