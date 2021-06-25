/*
 * ESP8266 Deep sleep mode example
 * Rui Santos 
 * Complete Project Details https://randomnerdtutorials.com
 */

#include <SPI.h>
#include <SD.h>
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <ESP8266WiFi.h>       //Libraries for Communication
#include <PubSubClient.h>
#include <string.h>
// #include <Ticker.h>
#include "ESP8266TimerInterrupt.h"

#include <TimeLib.h>
#include <WiFiUdp.h>

ESP8266Timer ITimer;

#define SD_SS_pin 15
bool SD_available = 1;

void sampleCallback();

////////////.....Initialise my Wifi.....////////////
const char* ssid = "NOKIA-405BD8BBECB9";
const char* password = "4921607895";
const char* mqtt_server = "192.168.1.133";          // pi wifi: 192.168.1.133 @ home network
//                                               // or pi eth: 192.168.1.123 @ home network
WiFiClient espClient;
PubSubClient client(espClient);

// NTP
  static const char ntpServerName[] = "au.pool.ntp.org";
  const int timeZone = 10;
  WiFiUDP Udp;
  unsigned int localPort = 8888;
  time_t getNtpTime();
  void sendNTPpacket(IPAddress &address);
  const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
  byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
//

long lastMsg = 0;
static uint16_t OFF_THRESH = 600;
bool bulbState = 1;


////////////.....Wifi functions.....//////////// 
void setup_wifi() {
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // connect udp 
  Udp.begin(localPort);
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");

}

void callback(char* topic, byte* payload, int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Parse for bulb state
  String payloadStr = String((char*)payload);
  if (payloadStr.indexOf("\"state\":\"ON\"") > 0)
    bulbState = 1;
  else
    bulbState = 0;

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // ... and resubscribe
      client.subscribe("zigbee2mqtt/bulb");
      // Now query the state of the bulb
      client.publish("zigbee2mqtt/bulb/get","{\"state\": \"\"}");
      // We should get a reply in the callback function
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 second");
      // Wait 1 seconds before retrying
      delay(1000);
    }
  }
}

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

/*-------- NTP code ----------*/

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

////////////.....SD Card.....////////////
void setupSDCard()
{
  if(!SD.begin(SD_SS_pin))
  {
    Serial.println("SD Card intialisation failed!");
    SD_available = 0;
  }
}

////////////.....Distance ranger.....////////////
VL53L1X ranger;
void setup_vl531x()
{
  Wire.begin();
  Wire.setClock(400000);
  ranger.setTimeout(500);
  int timeout = 100;
  Serial.println("Connecting to ranger");
  while (!ranger.init() && timeout > 0)
  {
    delay(10);
    timeout--;
  }
  if (!timeout)
  {
    Serial.println("Ranger connection failed");
    while(1); // hang here 
  }
  Serial.println("Ranger connection success");
      
  ranger.setDistanceMode(VL53L1X::Long);
  ranger.setMeasurementTimingBudget(50000); // Long distance mode timing budget
  ranger.startContinuous(50);

  // Get the state of the light currently
   
}

String getTimeStamp()
{
  String str = "";
  int digits;
  str += String(hour());
  digits = minute();
  str+= ":"; 
  if (digits < 10) {str += "0";}str += String(digits);
  digits = second();
  str+= ":";
  if (digits < 10) {str += "0";} str += String(digits);

  str += ",";
  str += String(day());
  str += ".";
  str += String(month());
  str += ".";
  str += String(year()); 

  return str;
}

void writeEvent(uint16_t reading, bool state)
{
  File datafile = SD.open("events.txt", FILE_WRITE);
  if (datafile)
  {
    String dataString = "";
    dataString += getTimeStamp();
    dataString += " -> ";
    dataString += String(state);
    dataString += ",";
    dataString += String(reading);
    datafile.println(dataString);

    datafile.println(dataString);
  }
}

void sampleCallback()
{
  Serial.println("Sampling");
  // read from the sensor
  ranger.read(); 
  bool desired_state =0;
  uint16_t this_read = ranger.ranging_data.range_mm;

  Serial.println(this_read);

  // Based on the reading, which state should we be in?
  if (this_read > OFF_THRESH) // Door is closed, we should be off
    desired_state = 0;
  else if (this_read < OFF_THRESH) // Door is open, we should be on
    desired_state = 1;

  // Ensure we are in the desired state
  if (bulbState != desired_state)
  {
    Serial.println("sensor event!");
    char topic[] = "zigbee2mqtt/bulb/set";
    bulbState = desired_state;

    // Decide which state to put us in
    switch (desired_state) 
    {
      case 1 :
        client.publish(topic, "{\"state\": \"ON\"}");
        break;
      case 0 :
        client.publish(topic, "{\"state\": \"OFF\"}");
        break;
    }
    if (SD_available)
      writeEvent(this_read,desired_state);
  }  
}

////////////.....Arduino functions.....////////////
void setup() {
  Serial.begin(115200);
  Serial.println("Setting up wifi");
  setup_wifi(); // setup wifi connections & mqtt FIRST 
  Serial.println("Setting up sensor");
  setup_vl531x(); // now setup the sensor
  Serial.println("Setting up SD card");
  setupSDCard();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Serial.println("Starting timer");
  // ITimer.attachInterruptInterval(1000000, sampleCallback);
}
int last_samp = 0;
void loop(){
  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // loop the mqtt connection
  int now = millis();
  if (now - last_samp > 1000)
  {
    sampleCallback();
    last_samp = now;
  }

}