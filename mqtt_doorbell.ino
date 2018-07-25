/*
MQTT based doorbell, made by Steven
*/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Define NodeMCU D3 pin to as temperature data pin of  DHT11
#define DOORBUTTON  D3
#define RINGER      D0

// Update these with values suitable for your network.
const char* ssid = "SSID";
const char* password = "PASSOWORD";
const char* mqtt_server = "MQTT-SERVER-IP";
const char* mqtt_button_pub = "home/doorbell/button"; //on when pressed
const char* mqtt_ring_ring_sub = "home/doorbell/ringer/ring"; //to ring the bell remote
const char* mqtt_ring_sub = "home/doorbell/ringer/set"; // to enable/disable the ringer
const char* mqtt_ring_pub = "home/doorbell/ringer"; //ringer status
const char* mqtt_ring_time_sub = "home/doorbell/ringer/time/set"; //change the ringtime in ms
const char* mqtt_ring_time_pub = "home/doorbell/ringer/time"; // current ringtime in ms

WiFiClient espClient;
PubSubClient client(espClient);
long lastPress = 0;
char msg[50];
int value = 0;
int ringerOn = 1;
int ringTheBell = 0;

// Variables will change:
int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin

// the following variables are unsigned long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
unsigned long bellStartTime = 0;      // When did the ringing start
unsigned long bellRingTime = 1000;   // ring the bell for 1 second.
unsigned long reconnect_at_time = 0;

void setup_wifi() {
   delay(100);
  // We start by connecting to a WiFi network
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);
      Serial.print(".");
    }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Command received at: ");
  Serial.println(topic);
  String mqttTopic = "";
  String message = "";
  // find something smart for different payload (== ON | OFF)
  // /ringer/set
  if(strcmp(topic, mqtt_ring_sub) == 0) {
    mqttTopic = mqtt_ring_pub;
    if(strcmp((char*)payload, "0") == 0) {
      Serial.println("Ringer set to Off");
      ringerOn = 0;
      message = "0";
    } 
    if(strcmp((char*)payload, "1") == 0) {
      Serial.println("Ringer set to On");
      ringerOn = 1;
      message = "1";
    }
  }
  // /ringer/time/set
  else if(strcmp(topic, mqtt_ring_time_sub) == 0) {
    mqttTopic = mqtt_ring_time_pub;
    //parse payload for data
    //sscanf((char*)payload, "%u", &bellRingTime); // alternative way
    //you may request current bellRingTime by setting time to '0'
    if (atoi( (char*)payload) > 0 ) {
      bellRingTime = atoi( (char*)payload);
    }
    message = bellRingTime;
/*
    Serial.print("Topic received: ");
    Serial.print(mqttTopic);
    Serial.print(": ");
    String pp = (char*)payload;
    Serial.println(pp);
 */  
  }
  // ringer/ring
  else if(strcmp(topic, mqtt_ring_ring_sub) == 0) {
    //tpc = tpc + mqtt_ring_ring_pub;
    ringthebell( true );
  }
  
  char sendmessage[50];
  char sendmqttTopic[50];
  message.toCharArray(sendmessage,50);
  mqttTopic.toCharArray(sendmqttTopic, 50);
  Serial.print(sendmqttTopic);
  Serial.print(sendmessage);
  client.publish(sendmqttTopic, sendmessage, true);
  Serial.println(" ..sent");
} //end callback

void reconnect() {
  // dont Loop until we're reconnected
  Serial.print("Attempting MQTT connection...");
  // Create a random client ID
  String clientId = "ESP8266Client-";
  clientId += String(random(0xffff), HEX);
  // Attempt to connect
  //if you MQTT broker has clientID,username and password
  //please change following line to    if (client.connect(clientId,userName,passWord))
  if (client.connect(clientId.c_str()))
  {
    Serial.println("connected");
   //once connected to MQTT broker, subscribe command if any
    client.subscribe(mqtt_ring_sub);
    client.subscribe(mqtt_ring_ring_sub);
    client.subscribe(mqtt_ring_time_sub);
  } else {
    Serial.print("failed, rc=");
    Serial.println(client.state());
  }
} //end reconnect()

void sendMQTTMessage(String topic, String message) {
    char msg[50];
    char tpc[50];
    topic.toCharArray(tpc, 50);
    message.toCharArray(msg,50);
    Serial.print(tpc);
    Serial.print(msg);
    //publish sensor data to MQTT broker
    client.publish(tpc, msg);
    Serial.println(" ..sent");
}

void ringthebell(boolean startRinging) {
  if (startRinging) {
    //we may ring the bell
    bellStartTime = millis(); //start the timer 
    if (ringerOn == 1){ //only when we want the ringer on
      digitalWrite(RINGER, HIGH);
    }
    sendMQTTMessage( mqtt_button_pub, "ON" ); // always send the message
  }

// check when to stop the ringer
  if ((millis() - bellStartTime) > bellRingTime ){
    // time has passed, stop ringing
    if (digitalRead(RINGER) == HIGH) {
      digitalWrite(RINGER, LOW);
      sendMQTTMessage( mqtt_button_pub, "OFF" );
    }
  }
}

int checkButtonPressed() {
  int reading = digitalRead(DOORBUTTON);
  if (reading != lastButtonState){
    //Serial.print("buttonchange detected: ");
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay){
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW){
        // actual button press detected
         lastButtonState = reading;
         return 1; // notify button press
      }
    }
  }
  lastButtonState = reading;
  return 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(DOORBUTTON, INPUT);
  pinMode(RINGER, OUTPUT);
  digitalWrite(RINGER, LOW); // LOW means ringer off
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Serial.print(" Starting..." );
}

void loop() {
  //Serial.print("reading:"+reading);

  if (!client.connected() && millis() > reconnect_at_time) {
    reconnect();
    //failed to reconnect?
    if (!client.connected()) {
      //try again 6seconds later
      Serial.println("Reconnect failed, trying again in 6 seconds");
      reconnect_at_time = millis()+6000;
    }
  }
  client.loop();

if ( checkButtonPressed() ){
  ringthebell( true );
}

// control the doorbell, every loop!
  ringthebell( false );
}
