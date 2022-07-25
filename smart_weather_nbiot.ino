/* Project :
   Send  Pm 2.5, Rain guage, moisture, temperature, EC, PH, Nitrogen, Phosphorus, Potassium data from 7 in 1 Sensor to NB-IOT
   Date : 11/7/2022
*/

#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include "REG_CONFIG.h"

#include <TimeLib.h>
#include <ArduinoJson.h>
#include "time.h"

WiFiManager wifiManager;

// OTA
#define HOSTNAME "SmartSensorNB-IOT02"
#define PASSWORD "12345678"

#define battPIN  34
#define donePIN  25

float Batt = 0.0;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600 * 7;

StaticJsonDocument<400> doc;
bool validEpoc = false;
bool connectWifi = false;
int nbErrorTime = 0;
unsigned long time_s = 0;
unsigned long _epoch = 0;
struct tm timeinfo;

String json = "";

HardwareSerial modbus(2);
HardwareSerial_NB_BC95 AISnb;

// instantiate ModbusMaster object
ModbusMaster node;

// thingcontrol.io setup
String deviceToken = "";
String serverIP = "147.50.151.130";
String serverPort = "19956";

struct pm2510_7in1
{
  String pm2510temp;
  String pm2510humi;
  String pm2510PM2_5;
  String pm2510PM10;
  String soilmoisture;
  String soiltemp;
  String soilEC;
  String soilPH;
  String soilNit;
  String soilPho;
  String soilPot;
  String rainaccumulate;
  String raincount;
  String battery;
};

pm2510_7in1 smartsensor ;

// NB
signal meta ;
String imsi = "";
String imei = "";

unsigned long currentMillis;
unsigned long previousMillis;
const unsigned long interval = 60000; // Interval Time

/*
  writeSingleRegister( uint16_t  u16WriteAddress, uint16_t  u16WriteValue )
  uint8_t result;
  uint16_t data[6];

  // Toggle the coil at address 0x0002 (Manual Load Control)
  result = node.writeSingleCoil(0x0002, state);
  state = !state;

  // Read 16 registers starting at 0x3100)
  result = node.readInputRegisters(0x3100, 16);
  if (result == node.ku8MBSuccess)
  {
    Serial.print("Vbatt: ");
    Serial.println(node.getResponseBuffer(0x04)/100.0f);
    Serial.print("Vload: ");
    Serial.println(node.getResponseBuffer(0xC0)/100.0f);
    Serial.print("Pload: ");
    Serial.println((node.getResponseBuffer(0x0D) +
                    node.getResponseBuffer(0x0E) << 16)/100.0f);
  }

*/

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting... Smart Sensor Station"));
  Serial.println();
  Serial.println(F("***********************************"));

  AISnb.debug = true;
  AISnb.setupDevice(serverPort);
  String ip1 = AISnb.getDeviceIP();
  pingRESP pingR = AISnb.pingIP(serverIP);
  deviceToken = AISnb.getNCCID();

  /* OTA
    wifiManager.setTimeout(180);
    wifiManager.setAPCallback(configModeCallback);
    wifiManager.setAPClientCheck(true);
    String wifiName = "@ESP32-";
    wifiName.concat(String((uint32_t)ESP.getEfuseMac(), HEX));
    if (!wifiManager.autoConnect(wifiName.c_str())) {
    //Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    //    ESP.reset();
    //delay(1000);
    ESP.restart();
    delay(1);
    }
    setupWIFI();
    setupOTA();

  */

  /*
    while (nbErrorTime < 10) {
    meta = AISnb.getSignal();
    Serial.print("meta.rssi:"); Serial.println(meta.rssi);
    if (!meta.rssi.equals("N/A")) {
     if (meta.rssi.toInt() > -110) {
       break;
     } else {
       nbErrorTime++;
       delay(1000);
     }
    } else {
     nbErrorTime++;
     delay(1000);
    }
    }

    if (nbErrorTime == 10) {
    connectWifi = true;
    }

    if (connectWifi == false) {
    json = "{\"_type\":\"retrattr\",\"Tn\":\"";
    json.concat(deviceToken);
    json.concat("\",\"keys\":[\"epoch\"]}");
    getEpoch(serverIP, serverPort, json);
    }
    if (connectWifi == true) {
    configTime(gmtOffset_sec, 0, ntpServer);
    }

  */
}

void loop() {
  //  ArduinoOTA.handle();
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    readSensor();
    sendViaNBIOT();
    resetRain();
//    delay(60000);
    previousMillis = currentMillis;
  }
  //t6CheckTime();
}

void resetRain()
{
  uint8_t j, result;
  uint16_t data;
  String dat;
  Serial.begin(115200);
  modbus.begin(2400, SERIAL_8N1, 16, 17);
  delay(20);
  dat = readModbus(ID_RAIN, Address_RDRGP05[0]);
  Serial.print("Rain Accumulate : ");  Serial.println(dat);
  Serial.println("---------------------------------------------------------");
  // communicate with Modbus slave ID 1 over Hardware Serial (port 1)
  node.begin(ID_RAIN, modbus);
  result = node.writeSingleRegister(0x0000, 90);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    data = node.getResponseBuffer(0);
    Serial.println("Data :");
    Serial.println(data);
  }
  delay(20);
  dat = readModbus(ID_RAIN, Address_RDRGP05[0]);
  Serial.print("Rain Accumulate : ");  Serial.println(dat);
  Serial.println("---------------------------------------------------------");
}

long readModbus(char addr, uint16_t  REG)
{
  uint8_t j, result;
  uint16_t data;
  // communicate with Modbus slave ID 1 over Hardware Serial (port 1)
  node.begin(addr, modbus);
  result = node.readHoldingRegisters(REG, 1);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    data = node.getResponseBuffer(0);
    //Serial.println("Connec modbus Ok.");
    return data;
  } else
  {
    Serial.print("Connec modbus ID: ");
    Serial.print(addr);
    Serial.print(" Sensor fail. REG >>> ");
    Serial.println(REG); // Debug
    delay(100);
    return 0;
  }
}

void readSensor()
{
  modbus.begin(9600, SERIAL_8N1, 16, 17);
  delay(300);
  smartsensor.pm2510PM2_5 = readModbus(ID_PM25, Address_PM2510[0]);
  delay(10);
  smartsensor.pm2510PM10 = readModbus(ID_PM25, Address_PM2510[1]);
  delay(10);
  smartsensor.pm2510temp = readModbus(ID_PM25, Address_PM2510[2]);
  delay(10);
  smartsensor.pm2510humi = readModbus(ID_PM25, Address_PM2510[3]);
  delay(10);
  Serial.println("---------------------------------------------------------");
  Serial.print("PM 2.5 : ");  Serial.println(smartsensor.pm2510PM2_5);
  Serial.print("PM 10 : ");  Serial.println(smartsensor.pm2510PM10);
  Serial.print("Temperature : ");  Serial.println(((smartsensor.pm2510temp.toFloat() / 100) - 40));
  Serial.print("Humidity : ");  Serial.println(smartsensor.pm2510humi.toFloat() / 100);
  Serial.println("---------------------------------------------------------");

  modbus.begin(4800, SERIAL_8N1, 16, 17);   //for seven in one
  delay(30);
  smartsensor.soilmoisture = readModbus(ID_SOIL, Address_7IN1[0]);
  delay(10);
  smartsensor.soiltemp = readModbus(ID_SOIL, Address_7IN1[1]);
  delay(10);
  smartsensor.soilEC = readModbus(ID_SOIL, Address_7IN1[2]);
  delay(10);
  smartsensor.soilPH = readModbus(ID_SOIL, Address_7IN1[3]);
  delay(10);
  smartsensor.soilNit = readModbus(ID_SOIL, Address_7IN1[4]);
  delay(10);
  smartsensor.soilPho = readModbus(ID_SOIL, Address_7IN1[5]);
  delay(10);
  smartsensor.soilPot = readModbus(ID_SOIL, Address_7IN1[6]);
  delay(10);
  Serial.print("Moisture : ");  Serial.println(smartsensor.soilmoisture);
  Serial.print("Temperature : ");  Serial.println(smartsensor.soiltemp);
  Serial.print("EC : ");  Serial.println(smartsensor.soilEC);
  Serial.print("PH : ");  Serial.println(smartsensor.soilPH);
  Serial.print("Nitrogen : ");  Serial.println(smartsensor.soilNit);
  Serial.print("Phosphorus : ");  Serial.println(smartsensor.soilPho);
  Serial.print("Potassium : ");  Serial.println(smartsensor.soilPot);
  Serial.println("---------------------------------------------------------");

  modbus.begin(2400, SERIAL_8N1, 16, 17);
  delay(20);
  smartsensor.rainaccumulate = readModbus(ID_RAIN, Address_RDRGP05[0]);
  delay(10);
  smartsensor.raincount = readModbus(ID_RAIN, Address_RDRGP05[1]);
  delay(10);
  Serial.print("Rain Accumulate : ");  Serial.println(smartsensor.rainaccumulate);
  Serial.print("Rain Count : ");  Serial.println(smartsensor.raincount);
  Serial.println("---------------------------------------------------------");

  //  smartsensor.battery = Read_Batt();
  //  Serial.print("Battery : "); Serial.print(smartsensor.battery); Serial.println(" V");

  Serial.print("RSSI : "); Serial.println(meta.rssi);
  Serial.println("---------------------------------------------------------");
}

void sendViaNBIOT() {
  meta = AISnb.getSignal();
  String json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"atem\":");
  json.concat(((smartsensor.pm2510temp.toFloat() / 100) - 40));
  json.concat(",\"ahum\":");
  json.concat(smartsensor.pm2510humi.toFloat() / 100);
  json.concat(",\"apm2.5\":");
  json.concat(smartsensor.pm2510PM2_5);
//  json.concat(",\"apm10\":");
//  json.concat(smartsensor.pm2510PM10);
  json.concat(",\"smoi\":");
  json.concat(smartsensor.soilmoisture.toFloat() / 10);
  json.concat(",\"stem\":");
  json.concat(smartsensor.soiltemp.toFloat() / 10);
//  json.concat(",\"sEC\":");
//  json.concat(smartsensor.soilEC);
  json.concat(",\"soil_PH\":");
  json.concat(smartsensor.soilPH.toFloat() / 10);
//  json.concat(",\"sN\":");
//  json.concat(smartsensor.soilNit);
//  json.concat(",\"sP\":");
//  json.concat(smartsensor.soilPho);
//  json.concat(",\"sK\":");
//  json.concat(smartsensor.soilPot);
//  json.concat(",\"racc\":");
//  json.concat(smartsensor.rainaccumulate);
//  json.concat(",\"rcou\":");
//  json.concat(smartsensor.raincount);
  //  json.concat(",\"battery\":");
  //  json.concat(smartsensor.battery);
  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat("}");
  Serial.println(json);
  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  UDPReceive resp = AISnb.waitResponse();
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void setupWIFI()
{
  WiFi.setHostname(HOSTNAME);
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count ++;
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connecting...OK.");
  else
    Serial.println("Connecting...Failed");
}

// OTA
void setupOTA()
{
  //Port defaults to 8266
  //ArduinoOTA.setPort(8266);
  //Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME);

  //No authentication by default
  ArduinoOTA.setPassword(PASSWORD);
  ArduinoOTA.onStart([]()
  {
    Serial.println("Start Updating....");
    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
  });
  ArduinoOTA.onEnd([]()
  {
    Serial.println("Update Complete!");
    ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    String pro = String(progress / (total / 100)) + "%";
    int progressbar = (progress / (total / 100));
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;
      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;
      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;
      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;
      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }
    Serial.println(info);
    ESP.restart();
  });
  ArduinoOTA.begin();
}


float Read_Batt()
{
  unsigned int vRAW = 0;
  float Vout = 0.0;
  float Vin = 0.0;
  float R1 = 15000.0;
  float R2 = 3900.0;
  int bitRes = 4096;
  int16_t adc = 0;
  for (int a = 0; a < 20; a++)
  {
    adc  += analogRead(battPIN);
    delay(1);
  }

  vRAW = adc / 20;
  Serial.println(vRAW);
  Vout = (vRAW * 3.3162) / bitRes;
  Serial.println(Vout);
  Vin = Vout / (R2 / (R1 + R2));
  if (Vin < 0.05)
  {
    Vin = 0.0;
  }
  return Vin;
}

char  char_to_byte(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (c - 0x30);
  }
  if ((c >= 'A') && (c <= 'F'))
  {
    return (c - 55);
  }
}

void getEpoch(String IP, String Port, String Data) {
  json = "";
  do {
    //    if (AISnb.pingIP(serverIP).status == false) {
    //      ESP.restart();
    //    }
    UDPSend udp = AISnb.sendUDPmsgStr(IP, Port, Data);

    //String nccid = AISnb.getNCCID();
    //Serial.print("nccid:");
    //Serial.println(nccid);

    UDPReceive resp = AISnb.waitResponse();
    AISnb.receive_UDP(resp);
    Serial.print("waitData:");
    Serial.println(resp.data);
    if (udp.status == false) {
      connectWifi = true;
      break;
    } else {
      for (int x = 0; x < resp.data.length(); x += 2) {
        char c = char_to_byte(resp.data[x]) << 4 | char_to_byte(resp.data[x + 1]);

        json += c;
      }
      //Serial.println(json);
      DeserializationError error = deserializeJson(doc, json);

      // Test if parsing succeeds.
      if (error) {
        //Serial.print(F("deserializeJson() failed: "));
        //Serial.println(error.f_str());
        validEpoc = true;
        delay(1000);
      } else {
        validEpoc = false;
        time_s = millis();
        _epoch = doc["epoch"];
        //SerialBT.println(json);
        Serial.println(json);
        //Serial.print("epoch:");
        //Serial.println(_epoch);
      }
    }
    //
  } while (validEpoc);
}

void t6CheckTime() {
  //Serial.println("Check Time");
  if (connectWifi == false) {
    Serial.print("epoc"); Serial.println(_epoch);
    Serial.print("time s:"); Serial.println(time_s);
    if (_epoch != 0 && (millis() - time_s) > 300000 && hour(_epoch + ((millis() - time_s) / 1000) + (7 * 3600)) == 0) {
      Serial.println("Restart");
      //ESP.restart();
    }
  } else {
    if (!getLocalTime(&timeinfo)) {
      //Serial.println("Failed to obtain time");
      return;
    }
    Serial.print("timeinfo.tm_hour:"); Serial.println(timeinfo.tm_hour);
    Serial.print("timeinfo.tm_min:"); Serial.println(timeinfo.tm_min);
    if (( timeinfo.tm_hour == 0) && ( timeinfo.tm_min < 10) ) {
      Serial.println("Restart @ midnight2");
      //ESP.restart();
    }
  }
}
