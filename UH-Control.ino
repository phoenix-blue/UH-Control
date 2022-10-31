#include <DS18B20.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>
#include <WiFi.h>
#include "PCF8574.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/************************* WiFi Access Point *********************************/

#define WLAN_SSID "SSID_NAME"   // enter your ssid
#define WLAN_PASS "123456789"   //enter your wifi password
// Adafruit IO
#define AIO_SERVER ""
#define AIO_SERVERPORT 1883
#define AIO_USERNAME ""
#define AIO_KEY ""
/************************* PCF8574 *********************************/

//#define I2C_RELAYS_ADR 0x24
//#define I2C_DIGITAL_ADR 0x22
PCF8574 pcf1(0x24, 4, 15);
PCF8574 pcf(0x22, 4, 15);
unsigned long current = 0;
unsigned long i = 0;
/************************* ds18b20 *********************************/
//#define ONE_WIRE_BUS 32
//OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature sensors(&oneWire);
DS18B20 ds1(32);  //channel-1-DS18b20
DS18B20 ds2(33);  //channel-2-DS18b20
int deviceCount = 0;
float tempC;

/************ Global State ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;


// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.


/****************************** Feeds ***************************************/
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish setvalue = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/setvalue");
Adafruit_MQTT_Publish sensor1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sensor1");
Adafruit_MQTT_Publish sensor2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sensor2");
Adafruit_MQTT_Publish waterpump = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/waterpump");
Adafruit_MQTT_Publish heatpump = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/heatpump");
Adafruit_MQTT_Publish boiler = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/boiler");
Adafruit_MQTT_Subscribe setpoint = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/setpoint", MQTT_QOS_1); //set point


/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();
uint16_t save;
float  a = 0;
float b = 0;
int f, g, h = 0;
//DS18B20 ds(32);
//DS18B20 ds1(33);

void setup() {
  Serial.begin(115200);
  delay(10);
  pcf.pinMode(P0, INPUT_PULLUP);   //ThemorState Signal
  pcf.pinMode(P1, INPUT_PULLUP);   //Defrot Signl
  pcf.pinMode(P2, INPUT_PULLUP);   //Alarm Signal

  pcf1.pinMode(P0, OUTPUT); //Heat Pump
  pcf1.pinMode(P1, OUTPUT); //Boiler
  pcf1.pinMode(P2, OUTPUT); //Water Pump
  pcf1.pinMode(P3, OUTPUT); //Heat Pump
  pcf1.pinMode(P4, OUTPUT); //Boiler
  pcf1.pinMode(P5, OUTPUT); //Water Pump

  pcf1.digitalWrite(P0, HIGH);
  pcf1.digitalWrite(P1, HIGH);
  pcf1.digitalWrite(P2, HIGH);
  pcf1.digitalWrite(P3, HIGH);
  pcf1.digitalWrite(P4, HIGH);
  pcf1.digitalWrite(P5, HIGH);



  if (pcf.begin() && pcf1.begin() )
  {
    Serial.println(F("Ok pcf"));
  }
  else
  {
    Serial.println(F("Error"));
  }
  //deviceCount = sensors.getDeviceCount();
  //Serial.println("Devices on one wire");
  //Serial.println(deviceCount);
  //EEPROM.begin(save);
  Serial.println(F("Adafruit MQTT Connection"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&setpoint);
}


float sliderval;

void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();


  //sliderval = EEPROM.read(save);
  a = ds1.getTempC();  //OUTSIDE TEMP
  b = ds2.getTempC() ; //Water TEMP
  Serial.println("temp 1");
  Serial.println(a);
  Serial.println("temp 2");
  Serial.println(b);

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(6000))) {
    if (subscription == &setpoint) {
      Serial.print(F("Got: "));
      Serial.println((char *)setpoint.lastread);
      sliderval = atoi((char *)setpoint.lastread);
    }
  }

  //EEPROM.write(save, sliderval);
  // Now we can publish stuff!
  Serial.print(F("\nSending  "));

  Serial.print("...");
  if (! sensor1.publish(a)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
    delay(3000);
  }
  if (! setvalue.publish(sliderval)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
    delay(3000);
  }

  if (!sensor2.publish(b)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
    delay(5000);
  }
  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds

  if (! mqtt.ping()) {
    mqtt.disconnect();
  }

  uint8_t val1 = pcf.digitalRead(P0);
  uint8_t val2 = pcf.digitalRead(P1);
  uint8_t val3 = pcf.digitalRead(P2);

  Serial.println("D1 Input");
  Serial.println(val1);
  Serial.println("D2 Input");
  Serial.println(val2);
  Serial.println("D3 Input");
  Serial.println(val3);
  int k = 5;           //Water Temp Value can be change from here:
  if (val1 == LOW) {

    pcf1.digitalWrite(P5, LOW);  //WaterPump ON
    f = 1;
    if (sliderval < a && k <= b)
    {
      pcf1.digitalWrite(P3, LOW);     //Heatpump
      g = 1;
    } else
    {
      pcf1.digitalWrite(P3, HIGH);
      g = 0;
    }

    if (sliderval >= a || val2 == LOW || val3 == LOW || b <= 5 )
    {
      current = 1;
      i = 0;
      h = 1;
      pcf1.digitalWrite(P4, LOW);  //boiler
    }
    else  {
      if (current == 1)
      {
        pcf1.digitalWrite(P4, LOW);
        i++;
      }
    }
    if (i >= 3) {    // for delay off time of biler can be change here 3 to 4,5,6 etc its mean 15 sec will be add
      pcf1.digitalWrite(P4, HIGH);
      h = 0;
      i = 0;
      current = 0;
    }
  } else if (val1 == HIGH)
  {

    f = 0; g = 0; h = 0;
    pcf1.digitalWrite(P3, HIGH);
    pcf1.digitalWrite(P4, HIGH);
    pcf1.digitalWrite(P5, HIGH);
  }
  if (! waterpump.publish(f)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
    delay(2000);
  }

  if (! heatpump.publish(g)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
    delay(2000);
  }

  if (! boiler.publish(h)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
    Serial.println("Free heap:");
    Serial.println(ESP.getFreeHeap());
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());
    Serial.print("Wifi status: ");
    Serial.println(WiFi.status());
    delay(2000);
  }


}
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 10;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 30 seconds...");
    mqtt.disconnect();
    delay(30000);  // wait 30 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}
