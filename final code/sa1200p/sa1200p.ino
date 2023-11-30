#include <SoftwareSerial.h>
#include <BME280I2C.h>
#include <Wire.h>

// web update
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

#ifndef STASSID
#define STASSID "DijckNoyens_24G"
#define STAPSK "f3sBas7cTVU7CTSu"
#endif

const char* host = "esp8266-bob-webupdate";
const char* ssid = STASSID;
const char* password = STAPSK;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;


#define SERIAL_BAUD 115200

BME280I2C bme;  // Default : forced mode, standby time = 1000 ms
                // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
float temp(NAN), hum(NAN), pres(NAN);

//#define DEBUG

const byte CO2_RX = 14;  // D5
int co2;

SoftwareSerial SerialCO2(CO2_RX);  // RX

void setup() {
  Serial.begin(SERIAL_BAUD);

  // doesn't work on my board (https://arduino.stackexchange.com/questions/65017/arduino-ide-while-serial)
  // while(!Serial)
  // {
  //   // Wait
  //   int a = 2;
  // }
  delay(2000);

  Serial.println("Serial ready!");

  Serial.printf("Starting network (WIFI, SSID [%s])!\n", ssid);

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    Serial.println("WiFi failed, retrying.");
  }

  MDNS.begin(host);

  httpUpdater.setup(&httpServer);
  httpServer.on("/data", web_handle_data);
  httpServer.onNotFound(web_handle_notfound);
  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser (or use IP '%s' instead of '%s' in URL)\n", host, WiFi.localIP().toString().c_str(), host);

  Wire.begin();

  while (!bme.begin()) {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  Serial.println("Found BME280 sensor!");

  switch (bme.chipModel()) {
    case BME280::ChipModel_BME280:
      Serial.println("Found BME280 sensor! Success.");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println("Found BMP280 sensor! No Humidity available.");
      break;
    default:
      Serial.println("Found UNKNOWN sensor! Error!");
  }

  SerialCO2.begin(9600);

  delay(500);
}

void loop() {
  if (SerialCO2.available()) {
    co2 = get_CO2();
    if (co2 == -1) {
      Serial.println("Serial data (co2) corrupt");
    } else {
      Serial.println(GetIpAddress() + " CO2 = " + String(co2) + " ppm");
    }
  }

  printBME280Data(&Serial);

  delay(100);

  httpServer.handleClient();
  MDNS.update();
}

// get the CO2 data
int get_CO2() {
  int co2_lvalue;
  byte co2_byte_counter = 0, co2_data[10], co2_checksum;
  while (SerialCO2.available()) {
    co2_data[co2_byte_counter] = SerialCO2.read();
    co2_byte_counter++;
  }
  // are the three first byte ok?
  if ((co2_data[0] != 0x16) || (co2_data[1] != 0x05) || (co2_data[2] != 0x01)) {
#ifdef DEBUG
    Serial.println("data corrupt");
#endif
    return -1;
  }
  // calculate the checksum and test if ok
  co2_checksum = 0;
  for (byte i = 0; i < co2_byte_counter - 1; i++) {
    co2_checksum += co2_data[i];
  }
  co2_checksum = 256 - co2_checksum;
  if (co2_data[co2_byte_counter - 1] != co2_checksum) {
#ifdef DEBUG
    Serial.println("Checksum not ok");
#endif
    return -1;
  }
  co2_lvalue = co2_data[3] * 256 + co2_data[4];
#ifdef DEBUG
  for (byte i = 0; i < co2_byte_counter; i++) {
    Serial.print(co2_data[i], HEX);
    Serial.print(' ');
  }
  Serial.print(" checksum: " + String(co2_checksum, HEX));
#endif
  for (byte i = 0; i < sizeof(co2_data); i++) {  //clear buffer
    co2_data[i] = 0;
  }
  co2_byte_counter = 0;
  return co2_lvalue;
}

String GetIpAddress() {
  return "[" + String(WiFi.localIP().toString().c_str()) + "]";
}

void printBME280Data(Stream* client) {

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(pres, temp, hum, tempUnit, presUnit);

  client->print(GetIpAddress() + " ");
  client->print("Temp: ");
  client->print(temp);
  client->print("°" + String(tempUnit == BME280::TempUnit_Celsius ? 'C' : 'F'));
  client->print("\t\tHumidity: ");
  client->print(hum);
  client->print("% RH");
  client->print("\t\tPressure: ");
  client->print(pres);
  client->println("Pa");

  delay(1000);
}

void web_handle_data() {
  Serial.println("data request, sending back to client...");
  char szResponse[512];

  const char* HtmlTemplate = "<!DOCTYPE html><html>"
                             "<head><title>SA1200P data</title><meta http-equiv=\"refresh\" content=\"10\"></head>"
                             "<body>"
                             "<table><tr>"
                             "<th>CO2</th>"
                             "<th>Temperature</th>"
                             "<th>Humidity</th>"
                             "<th>Pressure</th>"
                             "</tr>"
                             "<tr>"
                             "<td>%d</td>"
                             "<td>%.2f</td>"
                             "<td>%.2f</td>"
                             "<td>%.2f</td>"
                             "</tr>"
                             "</table>"
                             "</br>"
                             "<p>CO2: from SA1200P</br>"
                             "TEMP, HUM, PRESS: from added ESP8266 :-)</p>"
                             "<p>IP address: %s</p>"
                             "<small>Page reloads every 10 seconds</small>"
                             "</body></html>";

  sprintf(szResponse, HtmlTemplate, co2, temp, hum, pres, WiFi.localIP().toString().c_str());
  httpServer.send(200, "text/html", szResponse);
}

void web_handle_notfound() {
  char szResponse[256];

  const char* HtmlTemplate = "<!DOCTYPE html><html><body>"
                             "<p>Try <code>/data</code> or <code>/update</code></p>"
                             "</body></html>";

  sprintf(szResponse, HtmlTemplate, co2, temp, hum, pres, WiFi.localIP().toString().c_str());
  httpServer.send(200, "text/html", szResponse);
}