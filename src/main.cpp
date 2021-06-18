#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <ESP8266WiFi.h>       //Libraries for Communication
#include <PubSubClient.h>
#include <string.h>

////////////.....Initialise my Wifi.....////////////
const char* ssid = "NOKIA-405BD8BBECB9";
const char* password = "4921607895";
const char* mqtt_server = "192.168.1.133";          // pi wifi: 192.168.1.133 @ home network
//                                               // or pi eth: 192.168.1.123 @ home network
WiFiClient espClient;
PubSubClient client(espClient);


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

////////////.....Arduino functions.....////////////
void setup() {
  Serial.begin(115200);
  setup_wifi(); // setup wifi connections & mqtt FIRST 
  setup_vl531x(); // now setup the sensor
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop(){
  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // loop the mqtt connection
  ranger.read(); // read from the sensor
  bool desired_state;
  uint16_t this_read = ranger.ranging_data.range_mm;

  // Based on the reading, which state should we be in?
  if (this_read > OFF_THRESH) // Door is closed, we should be off
    desired_state = 0;
  else if (this_read < OFF_THRESH) // Door is open, we should be on
    desired_state = 1;

  // Change at most once per second
  if (bulbState != desired_state)
  {
    long now = millis();
    if (now - lastMsg > 1000) {
      lastMsg = now;
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
    }
  }  
  // Serial.println("Going to sleep for 5 minutes");
  // ESP.deepSleep(3e8); // 3e8 (us) = 5 mins
}