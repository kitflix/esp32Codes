#include <EEPROM.h>
#include <WiFi.h>
#include <LiquidCrystal.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WLAN_SSID       "ketaki"
#define WLAN_PASS       "ketaki123"
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "ketakisudame"
#define AIO_KEY         "aio_VeHj27iNc49lAb5jFnhuqmkt9Nz9"
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish volt_p = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/volt");
Adafruit_MQTT_Publish current_p = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/current");
Adafruit_MQTT_Publish power_p = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/power");
Adafruit_MQTT_Publish unit_p = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/unit");
Adafruit_MQTT_Publish bill_p = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bill");


// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe relay1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/relay1");
Adafruit_MQTT_Subscribe relay2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/relay2");

/*************************************************************************/
#define ANALOG_PIN_0 36
int currentPin = 36;              //Assign CT input to pin 1
float kilos = 0;
int peakPower = 0;

int mVperAmp = 185;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
/****************************************************************************/

unsigned long previousMillis = 0;
unsigned long interval = 120000;
float volt;
int myunit;
double oldmyunit;
int billamount = 0;

/*********************************************************/
void gsm_init()
{
  Serial2.println("AT"); //Once the handshake test is successful, it will back to OK
  delay(1000);
  Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
  delay(1000);
  Serial2.println("AT+CMGS=\"9004940011\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  delay(500);
  Serial2.print("IoT Based Energy Monitoring System"); //text content
  Serial2.write(26);
}
void gsm_sms(char *str)
{
  Serial2.println("AT"); //Once the handshake test is successful, it will back to OK
  delay(1000);
  Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
  delay(1000);
  Serial2.println("AT+CMGS=\"9004940011\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  delay(500);
  Serial2.print(str); //text content
  Serial2.write(26);
}
/*********************************************************/
float getVPPVolt()
{
  float result;

  int readValue;             //value read from the sensor
  int maxValue = 0;          // store max value here
  int minValue = 4096;          // store min value here

  uint32_t start_time = millis();
  while ((millis() - start_time) < 1000) //sample for 1 Sec
  {
    readValue = analogRead(39);
    // see if you have a new maxValue
    if (readValue > maxValue)
    {
      /*record the maximum sensor value*/
      maxValue = readValue;
    }
    if (readValue < minValue)
    {
      /*record the maximum sensor value*/
      minValue = readValue;
    }
  }

  // Subtract min from max
  result = ((maxValue - minValue) * 3.3) / 4096.0;
  result = maxValue - minValue;
  return result;
}

float getVPP()
{
  float result;

  int readValue;             //value read from the sensor
  int maxValue = 0;          // store max value here
  int minValue = 4096;          // store min value here

  uint32_t start_time = millis();
  while ((millis() - start_time) < 1000) //sample for 1 Sec
  {
    readValue = analogRead(ANALOG_PIN_0);
    // see if you have a new maxValue
    if (readValue > maxValue)
    {
      /*record the maximum sensor value*/
      maxValue = readValue;
    }
    if (readValue < minValue)
    {
      /*record the maximum sensor value*/
      minValue = readValue;
    }
  }

  // Subtract min from max
  result = ((maxValue - minValue) * 3.3) / 4096.0;

  return result;
}
LiquidCrystal lcd(23, 22, 21, 19, 18, 5);
int v = 0;

char sms[150];

void setup() {
  Serial.begin(9600);
  EEPROM.begin(8);
  Serial2.begin(9600); delay(100);
  gsm_init();
  delay(2000);
  lcd.begin(16, 2);
  lcd.clear();
  Serial.println("\n\nHello world\n\n");
  lcd.print("IoT Based");
  lcd.setCursor(0,1);
  lcd.print("Enery Monitoring");
  delay(3000);
  lcd.clear();

  pinMode(2,OUTPUT); pinMode(0,OUTPUT);
  digitalWrite(2,HIGH); digitalWrite(0,HIGH);
 
  /*while(1)
  {
    digitalWrite(2,HIGH); digitalWrite(0,HIGH); delay(1000); digitalWrite(0,LOW);digitalWrite(2,LOW); delay(1000);  
  }*/

  
  // put your setup code here, to run once:
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  mqtt.subscribe(&relay1);
  mqtt.subscribe(&relay2);
  lcd.clear();
  lcd.print("WIFI Connected");
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  delay(3000);
  lcd.clear();
  // INIT Value in eeprom to start with 1 --> 
  float abctemp = 1.1;
  //EEPROM.put(0,abctemp); delay(200);  EEPROM.commit(); delay(100);
  EEPROM.get(0,kilos);
  Serial.print("\n\n ENergy Units read from EEPROM ARE "); Serial.println(kilos);
  //if(myunit> 10000) { myunit= 0; EEPROM.write(0,myunit); }
  oldmyunit = kilos;

  
}

void loop() 
{
  // put your main code here, to run repeatedly:
  MQTT_connect();
   Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) 
  {
    if (subscription == &relay1) 
    {
      Serial.print(F("Got: "));
      Serial.println((char *)relay1.lastread);
      char *value = (char *)relay1.lastread;
      String temp =  String(value);
      temp.trim();
      if(temp == "ON")
      {
        digitalWrite(0,LOW);
      }
      if(temp == "OFF")
      {
        digitalWrite(0,HIGH);
      }
    }
    if (subscription == &relay2) 
    {
      char *value = (char *)relay2.lastread;
      String temp2  = String(value);
      temp2.trim();
      Serial.print(F("Got: "));
      Serial.println((char *)relay2.lastread);
      if(temp2 == "ON")
      {
        digitalWrite(2,LOW);
      }
      if(temp2 == "OFF")
      {
        digitalWrite(2,HIGH);
      }
    }
  }

   Voltage = getVPP();
  VRMS = (Voltage / 2.0) * 0.707;
  AmpsRMS = (VRMS * 1000) / mVperAmp;
  AmpsRMS = AmpsRMS - 0.13;
  if (AmpsRMS < 0.35) AmpsRMS = 0;
  //Serial.print(AmpsRMS);
  //Serial.println(" Amps RMS");
  //Serial.println(" *************************************************");
 
  float volt = getVPPVolt();
  if (volt < 500) volt = 0;
  else volt= volt / 6.32;

  if (volt < 150) volt = 0;

  if(volt == 0) AmpsRMS = 0;
  
  Serial.print("Voltage = ");
  Serial.print(volt);
  Serial.println(" V\n\n");
  
  Serial.print("Current = ");
  Serial.println(AmpsRMS);

  int RMSPower = volt * AmpsRMS;  //Calculates RMS Power Assuming Voltage 220VAC, change to 110VAC accordingly
  if (RMSPower > peakPower)
  {
    peakPower = RMSPower;
  }
  kilos = kilos + (RMSPower * (2.05 / 60 / 60 / 1000)); //Calculate kilowatt hours used

  
  billamount = kilos * 8;
  lcd.setCursor(0,0); lcd.print("V:"); lcd.print(volt); lcd.print("  "); lcd.setCursor(9,0); lcd.print("C:"); lcd.print(AmpsRMS); lcd.print("    ");
  lcd.setCursor(0,1); lcd.print("W:"); lcd.print(RMSPower); lcd.print("   "); lcd.setCursor(8,1); lcd.print("U:"); lcd.print(kilos); lcd.print("  ");
  
  
  if (!bill_p.publish(billamount)) 
  {
    Serial.println(F("Failed"));
  }
  else 
  {
    Serial.println(F("OK!"));
  }
  delay(2000);
  if (!volt_p.publish(volt)) 
  {
    Serial.println(F("Failed"));
  }
  else 
  {
    Serial.println(F("OK!"));
  }
  delay(2000);
  if (!current_p.publish(AmpsRMS)) 
  {
    Serial.println(F("Failed"));
  }
  else 
  {
    Serial.println(F("OK!"));
  }
  delay(2000);
  if (!power_p.publish(RMSPower)) 
  {
    Serial.println(F("Failed"));
  }
  else 
  {
    Serial.println(F("OK!"));
  }
  delay(2000);
  if (!unit_p.publish(kilos)) 
  {
    Serial.println(F("Failed"));
  }
  else 
  {
    Serial.println(F("OK!"));
  }
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) 
  {
    sprintf(sms, "Current:%f\nVoltage:%.1f\nPower: %d\nUnits: %f\nBill Amount=%d", AmpsRMS, volt, RMSPower,kilos,billamount);
    gsm_sms(sms);
    previousMillis = currentMillis;
    Serial.println("Test");
    Serial.println(previousMillis);
    Serial.println(kilos);
    EEPROM.put(0,kilos);
    
  }
}

void MQTT_connect()
{
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();

    delay(5000); // wait 5 seconds

    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }

  Serial.println("MQTT Connected!");
  
}
