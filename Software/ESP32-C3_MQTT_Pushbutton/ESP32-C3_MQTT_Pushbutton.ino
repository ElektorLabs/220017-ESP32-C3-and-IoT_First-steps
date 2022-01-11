
/* As this is a basic sample crendentials for WiFi and MQTT are hardcoded */
#define WIFI_SSID "changeme"
#define WIFI_PASS "changeme"

#define MQTT_SERVER "test.mosquitto.org"
#define MQTT_TOPIC_OUT "BUTTON"
#define MQTT_TOPIC_IN "RELAIS"

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


typedef enum {
  RED=0,
  BLUE,
  GREEN
} LedColor_t;


WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (256)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setLed( LedColor_t color, uint8_t value){
  switch(color){
    case RED:{
        ledcWrite(1,value);
    } break;

    case GREEN:{
       ledcWrite(2,value);
    } break;

    case BLUE:{
      ledcWrite(0,value);
    } break;

    default:{
      
    } break;
    
  }
}

void SetLedOn( void ){
  setLed(RED,16);
  setLed(BLUE,16);
  setLed(GREEN,16);
}

void SetLedOff(void){
  setLed(RED,0);
  setLed(BLUE,0);
  setLed(GREEN,0);
}

void ToggleLed( void ){
  if(0==ledcRead(0)){
    SetLedOn();
  } else {
    SetLedOff();
  }
}


void SetLedRGB(uint8_t R, uint8_t G, uint8_t B){
  setLed(RED,R);
  setLed(GREEN,G);
  setLed(BLUE,B);
}

void mqtt_reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32-C3-Switch";
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(MQTT_TOPIC_IN);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  StaticJsonDocument<0> filter;
  filter.set(true);
  StaticJsonDocument<32> doc;

  DeserializationError error = deserializeJson(doc, payload, length, DeserializationOption::Filter(filter));

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  bool state = doc["state"]; // false
  if(true==state){
    SetLedRGB(0,32,0);
  } else {
    SetLedRGB(32,0,0);
  }
}


void mqtt_send_button_pressed(){
  char buffer[256];
  StaticJsonDocument<32> doc;
  doc["pressed"] = true;
  size_t n = serializeJson(doc, buffer);
  client.publish(MQTT_TOPIC_OUT, (const unsigned char*)buffer, n,false);

}

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  //first setup the IOs of the ESP32-C3
  //We have an RGB LED on module connected to GPIO 6,3,4
  pinMode(5,OUTPUT); 
  pinMode(3,OUTPUT); 
  pinMode(4,OUTPUT);
  //As the led will be bright we use a PWM to control color and brigthness
  ledcSetup(0,10000,8);
  ledcSetup(1,10000,8);
  ledcSetup(2,10000,8);
  ledcAttachPin(5, 0);
  ledcAttachPin(3, 1);
  ledcAttachPin(4, 2);
  SetLedOn();
  delay(500);
  SetLedOff();
  //Next is to setup out push botton on GPIO 2
  pinMode(8,INPUT_PULLUP);  
  
  Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        ToggleLed();
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  SetLedOn();
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(callback);

  
  
}

void loop() {
  static uint32_t press_start=0;
  static uint32_t press_duration=0;
 

  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    mqtt_reconnect();
  } 
  client.loop();

    if(digitalRead(8)==0){
      if(press_start==0){
        press_start=millis();
        Serial.println("Press start");
      }
     
    }else {
      if(press_start>0){
        press_duration=millis()-press_start;
        press_start=0;
        Serial.println("Press end");
        Serial.print("Duration:");
        Serial.println(press_duration);
      }
    }
  
  if(press_duration>100){
    press_duration=0;
    //Button is pressed...send a mqtt message that we need to change the relais state
    Serial.println("Send MQTT message");
    mqtt_send_button_pressed();
  }
}
