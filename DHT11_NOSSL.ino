#include <dht.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <time.h>

// PINS SETUP
#define DHT11_PIN 2 //D4
#define Greenled 14//D5
#define Yellowled 12//D6
#define Redled 13//D7

//Wireless Config
#define ssid "GC-GE" //your WiFi Name
#define password "ISET+2018"   //Your Wifi Password

//MQTT CONFIG
#define mqtt_server "41.229.118.249"
#define port 33820
#define MSG_MAX_LENGTH 255
#define TEMP_SENSOR_UID "SENSOR_LhDxlW248StPYdg"  //Temp. Sensor UID
#define HUMID_SENSOR_UID "SENSOR_xPFcA9QfqDBb9Il" //Humidity Sensor UID
#define GatewayID "ESP8266-CLIENT1056"            //MQTTClientID
#define mqttTopic "/home/room/hamza"
#define mqttusername "hamzajeljeli"
#define mqttpassword "21545049H@m"
#define SENDING_INTERVAL 60                       //Seconds

//MISC
#define BLINK 100

dht DHT;
WiFiClient espClient;
PubSubClient client(espClient);
struct tm timeinfo;
StaticJsonBuffer<600> JSONbuffer;
char str[8];
char msg[MSG_MAX_LENGTH];
int chk = 0;

void setup() {
  pinMode(Greenled, OUTPUT);
  pinMode(Yellowled, OUTPUT);
  pinMode(Redled, OUTPUT);
  Serial.begin(9600);
  digitalWrite(Greenled, HIGH);
  client.setServer(mqtt_server, port);
  client.setCallback(callback);
  
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
}

void loop()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    if (client.connected())
    {
      JsonObject& JSONencoder = JSONbuffer.createObject();
      Serial.println("Begin Publishing");
      JSONencoder["GID"] = GatewayID;
      digitalWrite(Yellowled, LOW);
      delay(BLINK);
      digitalWrite(Yellowled, HIGH);
      delay(BLINK);
      /**
         Temp Message Creation
      */
      chk = DHT.read11(DHT11_PIN);
      delay(2000);
      //dtostrf(DHT.temperature, 6, 2, str);
      //trimStr(str);
      JSONencoder["SID"] = TEMP_SENSOR_UID;
      JSONencoder["TD"] = getCurrTime();
      JSONencoder["VAL"] = DHT.temperature;
      JSONencoder.printTo(msg, sizeof(msg));
      if (client.publish(mqttTopic, msg))
      {
        Serial.println("1- Sent");
      }
      else
      {
        Serial.println("1- Not Sent");
      }
      /**
         END.
         Humidity Message Creation
      */
      //dtostrf(DHT.humidity, 6, 2, str);
      //trimStr(str);
      JsonObject& JSONencoder2 = JSONbuffer.createObject();
      JSONencoder2["GID"] = GatewayID;
      JSONencoder2["SID"] = HUMID_SENSOR_UID;
      JSONencoder2["TD"] = getCurrTime();
      JSONencoder2["VAL"] = DHT.humidity;
      JSONencoder2.printTo(msg, sizeof(msg));
      if (client.publish(mqttTopic, msg))
      {
        Serial.println("2- Sent");
      }
      else
      {
        Serial.println("2- Not Sent");
      }
      /**
         END.
      */
      digitalWrite(Yellowled, LOW);
      delay(BLINK);
      digitalWrite(Yellowled, HIGH);
      //client.endPublish();
      Serial.println("Done Publishing");
      //client.disconnect();
      JSONbuffer.clear();
      delay(SENDING_INTERVAL * 1000);
      client.loop();
      
    }
    else
    {
      connectToBroker();
      getCurrTime();
    }
  }
  else
  {
    digitalWrite(Redled, HIGH);
    digitalWrite(Yellowled, HIGH);
    delay(1000);
    digitalWrite(Redled, LOW);
    digitalWrite(Yellowled, LOW);
    connectToWifi();
  }
}

boolean connectToWifi()
{
  boolean ret = false;
  delay(10);
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.print(" ");
  WiFi.begin(ssid, password);
  int timeout = 30; //Seconds
  while ((WiFi.status() != WL_CONNECTED) && (timeout > 0)) {
    digitalWrite(Yellowled, HIGH);
    delay(250);
    digitalWrite(Yellowled, LOW);
    delay(250);
    timeout--;
    Serial.print(".");
    digitalWrite(Yellowled, HIGH);
    delay(250);
    digitalWrite(Yellowled, LOW);
    delay(250);
  }
  switch (WiFi.status())
  {
    case WL_CONNECTED : ret = true; digitalWrite(Yellowled, HIGH); Serial.println(" Success !"); Serial.print("Got IP address : "); Serial.println(WiFi.localIP()); break; //Wifi is connected
    default: Serial.println((String)" Fail !\nError code is " + getWifiError(WiFi.status()) + " ."); break;
  }
  return ret;
}

String getWifiError(wl_status_t stat) {
  switch (stat)
  {
    case WL_NO_SHIELD : return "WL_NO_SHIELD";
    case WL_IDLE_STATUS  : return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL : return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED : return "WL_SCAN_COMPLETED";
    case WL_CONNECT_FAILED : return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST : return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED : return "WL_DISCONNECTED";
    case WL_CONNECTED : return "WL_CONNECTED";
defaut: return "UNKNOWN";
  }
}

/**
   Beginning MQTT Specific Methods
*/

void callback(char* topic, byte* payload, unsigned int length) {
  digitalWrite(Redled, LOW);
  delay(BLINK);
  Serial.print("Message arrived from broker on topic [");
  Serial.print(topic);
  Serial.print("] : ");
  digitalWrite(Redled, HIGH);

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

boolean connectToBroker() {
  // Loop until we're reconnect
  boolean resp = false;
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect(GatewayID, mqttusername, mqttpassword)) {
      digitalWrite(Redled, HIGH);
      Serial.println("Connected to broker !");
      resp = true;
    } else {
      Serial.print("Failed, Error code is (");
      Serial.print(client.state());
      Serial.print("). Trying again in 5,");
      digitalWrite(Redled, HIGH);
      delay(500);
      digitalWrite(Redled, LOW);
      delay(500);
      Serial.print(" 4,");
      digitalWrite(Redled, HIGH);
      delay(500);
      digitalWrite(Redled, LOW);
      delay(500);
      Serial.print(" 3,");
      digitalWrite(Redled, HIGH);
      delay(500);
      digitalWrite(Redled, LOW);
      delay(500);
      Serial.print(" 2,");
      digitalWrite(Redled, HIGH);
      delay(500);
      digitalWrite(Redled, LOW);
      delay(500);
      Serial.print(" 1,");
      digitalWrite(Redled, HIGH);
      delay(500);
      digitalWrite(Redled, LOW);
      delay(500);
      Serial.println(" Retrying ...");
      Serial.println("");
    }
  }
  return resp;
}

char* getCurrTime()
{
  int timeout = 30;
  time_t now = time(nullptr);
  while ((now < 8 * 3600 * 2) && (timeout > 0)) {
    delay(1000);
    now = time(nullptr);
    timeout--;
  }
  if (timeout <= 0)
  {
    Serial.println("Couldn't fetch time from remote server. Crashing.");
    while(1);
  }
  gmtime_r(&now, &timeinfo);
  char res[64]; //make this big enough to hold the resulting string
  strftime(res, sizeof(res), "%Y/%m/%d %H:%M", &timeinfo);
  return res;
}

void trimStr(char * s) {
  char * p = s;
  int l = strlen(p);
  while (isspace(p[l - 1])) p[--l] = 0;
  while (* p && isspace(* p)) ++p, --l;
  memmove(s, p, l + 1);
}
