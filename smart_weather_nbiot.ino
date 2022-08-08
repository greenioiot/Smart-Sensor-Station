/* Project :
   Send  Pm 2.5, Rain guage, moisture, temperature, EC, PH, Nitrogen, Phosphorus, Potassium data from 7 in 1 Sensor to NB-IOT
   Date : 11/7/2022
*/

#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"
#include "REG_CONFIG.h"
#include "BluetoothSerial.h"

#define battPIN  34
#define donePIN  25

float Batt = 0.0;

HardwareSerial modbus(2);
HardwareSerial_NB_BC95 AISnb;
BluetoothSerial SerialBT;
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

unsigned long currentMillis1, currentMillis2;
unsigned long previousMillis1, previousMillis2;
const unsigned long interval1 = 60000; // Interval Time
const unsigned long interval2 =120000; // Interval Time

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting... Smart Sensor Station"));
  Serial.println();
  Serial.println(F("***********************************"));
  SerialBT.begin("Smart Sensor Station");
  AISnb.debug = true;
  AISnb.setupDevice(serverPort);
  String ip1 = AISnb.getDeviceIP();
  pingRESP pingR = AISnb.pingIP(serverIP);
  deviceToken = AISnb.getNCCID();
}

void loop() {
  currentMillis1 = millis();
  if (currentMillis1 - previousMillis1 >= interval1)
  {
    readSensor();
    delay(1000);
    sendViaNBIOT1();
    delay(30000);
    sendViaNBIOT2();
    previousMillis1 = currentMillis1;
  }
  currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 >= interval2)
  {
    resetRain();
    delay(2000);
    sleepProcess();
    previousMillis2 = currentMillis2;
  }

}

void resetRain()
{
  uint8_t j, result;
  uint16_t data;
  String dat;
  // communicate with Modbus slave ID 1 over Hardware Serial (port 1)
  node.begin(ID_RAIN, modbus);
  result = node.writeSingleRegister(0x0000, 90);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    Serial.println("Reset is successful");
  } else
  {
    Serial.println("Reset is failed");
  }
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

  SerialBT.println("---------------------------------------------------------");
  SerialBT.print("PM 2.5 : ");  SerialBT.println(smartsensor.pm2510PM2_5);
  SerialBT.print("PM 10 : ");  SerialBT.println(smartsensor.pm2510PM10);
  SerialBT.print("Temperature : ");  SerialBT.println(((smartsensor.pm2510temp.toFloat() / 100) - 40));
  SerialBT.print("Humidity : ");  SerialBT.println(smartsensor.pm2510humi.toFloat() / 100);
  SerialBT.println("---------------------------------------------------------");

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

  SerialBT.print("Moisture : ");  SerialBT.println(smartsensor.soilmoisture);
  SerialBT.print("Temperature : ");  SerialBT.println(smartsensor.soiltemp);
  SerialBT.print("EC : ");  SerialBT.println(smartsensor.soilEC);
  SerialBT.print("PH : ");  SerialBT.println(smartsensor.soilPH);
  SerialBT.print("Nitrogen : ");  SerialBT.println(smartsensor.soilNit);
  SerialBT.print("Phosphorus : ");  SerialBT.println(smartsensor.soilPho);
  SerialBT.print("Potassium : ");  SerialBT.println(smartsensor.soilPot);
  SerialBT.println("---------------------------------------------------------");

  modbus.begin(2400, SERIAL_8N1, 16, 17);
  delay(20);
  smartsensor.rainaccumulate = readModbus(ID_RAIN, Address_RDRGP05[0]);
  delay(10);
  smartsensor.raincount = readModbus(ID_RAIN, Address_RDRGP05[1]);
  delay(10);
  Serial.print("Rain Accumulate : ");  Serial.println(smartsensor.rainaccumulate);
  Serial.print("Rain Count : ");  Serial.println(smartsensor.raincount);
  Serial.println("---------------------------------------------------------");

  SerialBT.print("Rain Accumulate : ");  SerialBT.println(smartsensor.rainaccumulate);
  SerialBT.print("Rain Count : ");  SerialBT.println(smartsensor.raincount);
  SerialBT.println("---------------------------------------------------------");
  smartsensor.battery = Read_Batt();
  Serial.print("Battery : "); Serial.print(smartsensor.battery); Serial.println(" V");
  SerialBT.print("Battery : "); SerialBT.print(smartsensor.battery); SerialBT.println(" V");
  Serial.print("RSSI : "); Serial.println(meta.rssi);
  SerialBT.print("RSSI : "); SerialBT.println(meta.rssi);
  Serial.println("---------------------------------------------------------");
  SerialBT.println("---------------------------------------------------------");
}

void sendViaNBIOT1() {
  meta = AISnb.getSignal();
  String json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"aTemp\":");
  json.concat(((smartsensor.pm2510temp.toFloat() / 100) - 40));
  json.concat(",\"aHum\":");
  json.concat(smartsensor.pm2510humi.toFloat() / 100);
  json.concat(",\"PM25\":");
  json.concat(smartsensor.pm2510PM2_5);
  json.concat(",\"PM10\":");
  json.concat(smartsensor.pm2510PM10);
  json.concat(",\"sMois\":");
  json.concat(smartsensor.soilmoisture.toFloat() / 10);
  json.concat(",\"sTemp\":");
  json.concat(smartsensor.soiltemp.toFloat() / 10);
  json.concat(",\"bat\":");
  json.concat(smartsensor.battery);
  //  json.concat(",\"rssi\":");
  //  json.concat(meta.rssi);
  json.concat("}");
  Serial.println(json);
  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  UDPReceive resp = AISnb.waitResponse();
}

void sendViaNBIOT2() {
  meta = AISnb.getSignal();
  String json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"sEC\":");
  json.concat(smartsensor.soilEC);
  json.concat(",\"sPH\":");
  json.concat(smartsensor.soilPH.toFloat() / 10);
  json.concat(",\"sN\":");
  json.concat(smartsensor.soilNit);
  json.concat(",\"sP\":");
  json.concat(smartsensor.soilPho);
  json.concat(",\"sK\":");
  json.concat(smartsensor.soilPot);
  json.concat(",\"rAcc\":");
  json.concat(smartsensor.rainaccumulate);
  json.concat(",\"rCou\":");
  json.concat(smartsensor.raincount);
  //  json.concat(",\"bat\":");
  //  json.concat(smartsensor.battery);
  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat("}");
  Serial.println(json);
  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  UDPReceive resp = AISnb.waitResponse();
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
  adc = analogRead(battPIN);
  //  for (int a = 0; a < 20; a++)
  //  {
  //    adc  += analogRead(battPIN);
  //    delay(1);
  //  }
  //  vRAW = adc / 20;
  vRAW = adc;
  Serial.println(vRAW);
  //  Vout = (vRAW * 3.3162) / bitRes;
  //  Serial.println(Vout);
  //  Vin = Vout / (R2 / (R1 + R2));
  Vin = adc * 0.004353019;
  if (Vin < 0.05)
  {
    Vin = 0.0;
  }
  return Vin;
}

void sleepProcess()
{
  pinMode(donePIN, OUTPUT);
  digitalWrite(donePIN, HIGH);
  delay(100);
}
