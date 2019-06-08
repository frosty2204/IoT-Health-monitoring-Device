
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <WiFi.h>
#include <PubSubClient.h>
#define REPORTING_PERIOD_MS     1500
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "OneButton.h"
OneButton button1(36, true);
String mac;
bool deviceConnected = false;
bool deviceDisconnected = true;
bool oldDeviceConnected;
PulseOximeter pox;
 char heartstring[4];
 char spostring[4];
 char tempstring[4];
 int val=0;
uint32_t tsLastReport = 0;
const char* ssid = "DK2_JIO";//"Distrack_v4.31"DK2_JIO
const char* password =  "ituly1234";//"krynet_80877"ituly1234
const char* mqttServer = "m16.cloudmqtt.com";
const int mqttPort = 19418;
const char* mqttUser = "ddbfoeoe";
const char* mqttPassword = "BQRtXmNjwIeK";
WiFiClient espClient;
PubSubClient client(espClient);

// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
    Serial.println("Beat!");
}
#define heartRateService BLEUUID((uint16_t)0x180D)
BLECharacteristic heartRateMeasurementCharacteristics(BLEUUID((uint16_t)0x2A8D), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sposCharacteristic(BLEUUID((uint16_t)0x2A62), BLECharacteristic::PROPERTY_READ);
BLECharacteristic tempCharacteristic(BLEUUID((uint16_t)0x2A20), BLECharacteristic::PROPERTY_READ);
BLEDescriptor heartRateDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor sposDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor tempDescriptor(BLEUUID((uint16_t)0x2901));


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
void InitBLE() {
  BLEDevice::init("PortableHealthDevice");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pHeart = pServer->createService(heartRateService);

  pHeart->addCharacteristic(&heartRateMeasurementCharacteristics);
  heartRateDescriptor.setValue("HeartRate");
  heartRateMeasurementCharacteristics.addDescriptor(&heartRateDescriptor);
  heartRateMeasurementCharacteristics.addDescriptor(new BLE2902());

  pHeart->addCharacteristic(&sposCharacteristic);
  sposDescriptor.setValue("SPO2");
  sposCharacteristic.addDescriptor(&sposDescriptor);
  //sposCharacteristic.addDescriptor(new BLE2902());

    pHeart->addCharacteristic(&tempCharacteristic);
  tempDescriptor.setValue("Temperature");
  tempCharacteristic.addDescriptor(&tempDescriptor);
 // tempCharacteristic.addDescriptor(new BLE2902());

  pServer->getAdvertising()->addServiceUUID(heartRateService);

  pHeart->start();
  // Start advertising
  pServer->getAdvertising()->start();
}
void alertButton() {
  char switchstring[4]="A";
  client.publish("health/switch", switchstring);
}
 
  //tsLastReport = millis();



void setup() {
  pinMode(39,INPUT);
  button1.attachDuringLongPress(alertButton);
  Serial.begin(115200);

  // Create the BLE Device
  InitBLE();
  Serial.println("Waiting a client connection to notify...");
   WiFi.enableSTA(true);
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
   mac=WiFi.macAddress();
  Serial.println("Connected to the WiFi network");
 
  client.setServer(mqttServer, mqttPort);
 
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
 
      Serial.println("connected");
 
    } else {
 
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  
  }
  Serial.print("Initializing pulse oximeter..");

    // Initialize the PulseOximeter instance
    // Failures are generally due to an improper I2C wiring, missing power supply
    // or wrong target chip
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }

    // The default current for the IR LED is 50mA and it could be changed
    //   by uncommenting the following line. Check MAX30100_Registers.h for all the
    //   available options.
    // pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

    // Register a callback for the beat detection
    pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop() {
    // notify changed value
    pox.update(); 
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& msg = jsonBuffer.createObject();
    val = analogRead(39);
uint32_t mv = ( val/2048.0)*3300; 
uint32_t cel = mv*0.1;
uint32_t farh = (cel*9)/5 + 32;
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        Serial.print("Heart rate:");
        Serial.print(pox.getHeartRate());
        Serial.print("bpm / SpO2:");
        Serial.print(pox.getSpO2());
        Serial.println("%");
        itoa(pox.getHeartRate(),heartstring,10);
        msg["heartbeat"]=heartstring;
         uint32_t hb = pox.getHeartRate();
         uint32_t spo = pox.getSpO2();
         itoa(pox.getSpO2(),spostring,10);
          msg["spO2"]=spostring;
         msg["mac"]=mac;
         itoa(farh,tempstring,10);
          msg["temprature"]=tempstring;
         char messageBuffer[100];
  msg.printTo(messageBuffer, sizeof(messageBuffer));
  Serial.println(messageBuffer);
 
         client.publish("health", messageBuffer);
         
          button1.tick();
  
    if (deviceConnected) {
      
       heartRateMeasurementCharacteristics.setValue((uint8_t*)&hb, 4);
  heartRateMeasurementCharacteristics.notify();

  sposCharacteristic.setValue((uint8_t*)&spo,4);
// sposCharacteristic.notify();

  tempCharacteristic.setValue((uint8_t*)&farh,4);
 //tempCharacteristic.indicate();
//        pCharacteristic2->setValue(spostring);
//        pCharacteristic2->notify();
//        
      
        
    } 
         tsLastReport = millis();}
    
}
