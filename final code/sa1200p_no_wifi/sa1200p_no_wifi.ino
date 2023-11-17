#include <SoftwareSerial.h>
#include <BME280I2C.h>
#include <Wire.h>

#define SERIAL_BAUD 115200

BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

//#define DEBUG

const byte CO2_RX = 14; // D5
int co2;

SoftwareSerial SerialCO2(CO2_RX); // RX

void setup() 
{
  Serial.begin(SERIAL_BAUD);

  while(!Serial) 
  {
    // Wait
  } 

  Wire.begin();

  Serial.println("Serial ready!");

  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  Serial.println("Found BME280 sensor!");

  switch(bme.chipModel())
  {
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
  
  Serial.println("Hi");
}
  
void loop() 
{ 
  if (SerialCO2.available())
  {    
    co2 = get_CO2();
    if (co2 == -1) {
      Serial.println("Serial data (co2) corrupt");
    }
    else {
      Serial.println("CO2 = " + String(co2) + " ppm");
    }    
  }  

  printBME280Data(&Serial);
   
  delay(100);
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
  for (byte i=0; i<co2_byte_counter-1; i++) {
    co2_checksum += co2_data[i];
  }
  co2_checksum = 256-co2_checksum;
  if (co2_data[co2_byte_counter-1]!=co2_checksum) {
    #ifdef DEBUG
      Serial.println("Checksum not ok");  
    #endif    
    return -1;
  }  
  co2_lvalue = co2_data[3]*256+co2_data[4];
  #ifdef DEBUG
    for (byte i=0; i<co2_byte_counter; i++) {    
      Serial.print(co2_data[i],HEX);  
      Serial.print(' ');  
    }  
    Serial.print(" checksum: " + String(co2_checksum,HEX));  
  #endif  
  for (byte i=0; i<sizeof(co2_data); i++) { //clear buffer
    co2_data[i]=0;    
  }  
  co2_byte_counter = 0;
  return co2_lvalue;
}
  
void printBME280Data(Stream* client)
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   client->print("Temp: ");
   client->print(temp);
   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
   client->print("\t\tHumidity: ");
   client->print(hum);
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(pres);
   client->println("Pa");

   delay(1000);
}
