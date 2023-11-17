// sa1200p_with_esp_no_wifi.ino
// weigu.lu

#include <SoftwareSerial.h>

//#define DEBUG

const byte CO2_RX = 14; // D5
const byte SDA_PIN = 4; // D2
const byte SCL_PIN = 5; // D1
byte event_counter = 0;
int co2;
float sht21_value;
float hum, temp;
const byte buffer_bytes = 8;
volatile byte bit_buffer[buffer_bytes];
volatile byte bit_counter = 0;  

SoftwareSerial SerialCO2(CO2_RX); // RX

void setup() {
  pinMode(SDA_PIN,INPUT);
  pinMode(SCL_PIN,INPUT);
  Serial.begin(115200);
  SerialCO2.begin(9600);
  delay(500);    
  Serial.println("Hi");
  attachInterrupt(digitalPinToInterrupt(SCL_PIN), isr_scl, RISING);
  noInterrupts();  
  clear_buffer(buffer_bytes);
  bit_counter = 0;  
  interrupts(); // allow interrupts
}
  
void loop() {    
  if (SerialCO2.available()) {    
    co2 = get_CO2();
    if (co2 == -1) {
      Serial.println("Serial data (co2) corrupt");
    }
    else {
      Serial.println("CO2 = " + String(co2) + " ppm");
    }    
  }  
  sht21_value = get_sht21_data();
  if ((event_counter == 2) && (sht21_value > 0.0)) {
    hum = sht21_value;
    Serial.println("Humidity is: " + String(hum) + "%");
  }
  else if ((event_counter == 3) && (sht21_value > 0.0)) {
    temp = sht21_value;
    Serial.println("Temperature is: " + String(temp) + "°C");
    event_counter = 0;
  }    
  else if ((sht21_value == -1)) {
    Serial.println("SHT21 CRC error"); 
  }    
  
  delay(100);
  yield();
}

ICACHE_RAM_ATTR void isr_scl() {  
  //delayMicroseconds(15);  
  if (digitalRead(SDA_PIN)) {
    bitSet(bit_buffer[bit_counter/8], 7-bit_counter%8);    
  }
  else {
    bitClear(bit_buffer[bit_counter/8], 7-bit_counter%8);    
  }
  bit_counter++;
}

//clear buffer
void clear_buffer(byte lbuffer_bytes) {
  for (byte i=0; i<lbuffer_bytes; i++) {
    bit_buffer[i]=0;    
  }
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
  
// we need global vars bit_counter, bitbuffer, envent_counter
float get_sht21_data() {
  byte msb, lsb, crc;
  unsigned int data_word = 0;
  float lhum, ltemp;
  if (bit_counter >= 28) {
    delay(2); //wait 2ms for the other bits
    noInterrupts();   
    if (bit_buffer[0]==0x80) { // master
      event_counter = 1;
    }
    else if (event_counter == 1) { // hum data from slave
      msb = bit_buffer[0];
      lsb = bit_buffer[1] << 1;    // eliminate ACK bits
      crc = bit_buffer[2] << 2;      
      lsb = lsb + (bit_buffer[2] >> 7);       
      crc = crc + (bit_buffer[3] >> 6);      
      if (check_crc(msb, lsb, crc) == false) {
        event_counter = 0;
        return -1;      
      }
      lsb &= 0xFC;                 // clear last 2 bits (data sheet)
      data_word = msb*256+lsb;
      lhum = -6.0+(125.0*data_word/65536.0);      
      check_crc(0x66, 0xA4, 0x37);      
      #ifdef DEBUG        
        Serial.print("msb: " + String(msb,HEX) + " lsb: " + String(lsb,HEX));
        Serial.println(" crc: " + String(crc,HEX) + " data word: " + String(data_word));
      #endif  
      event_counter = 2;      
    }
    else if (event_counter == 2) { // temp data from slave
            msb = bit_buffer[0];
      lsb = bit_buffer[1] << 1;    // eliminate ACK bits
      crc = bit_buffer[2] << 2;      
      lsb = lsb + (bit_buffer[2] >> 7);       
      crc = crc + (bit_buffer[3] >> 6);      
      if (check_crc(msb, lsb, crc) == false) {
        event_counter = 0;
        return -1;      
      }
      lsb &= 0xFC;                 // clear last 2 bits (data sheet)
      data_word = msb*256+lsb;
      ltemp = -46.85+(175.72*data_word/65536.0); 
      ltemp = ltemp -1.65;          // correction for housing error?
      #ifdef DEBUG
        Serial.print("msb: " + String(msb,HEX) + " lsb: " + String(lsb,HEX));
        Serial.println(" crc: " + String(crc,HEX) + " data word: " + String(data_word));
      #endif  
      event_counter = 3;
    }
    #ifdef DEBUG
      Serial.print(String(event_counter) + ":  " + String(bit_counter) + " bit:  ");
      for (byte i=0; i<buffer_bytes; i++) {
        Serial.print(bit_buffer[i],HEX);
        Serial.print('\t');    
      }
      delay(10);
      Serial.println();
    #endif  
    clear_buffer(buffer_bytes);
    bit_counter = 0;    
    interrupts();  
    if (event_counter == 2) {
      return lhum;
    }
    else if (event_counter == 3) {
      return ltemp;
    }        
  }
}

// from Sensiron application note and code
bool check_crc(byte lmsb, byte llsb, byte lcrc) {  
  byte bitmask;  
  byte calc_crc = 0x00; // initial value as per Table 17  
  calc_crc ^= (lmsb); // do msb
  for (bitmask = 8; bitmask > 0; --bitmask)  {
    if (calc_crc & 0x80) {
      calc_crc = (calc_crc << 1) ^ 0x131;
    }
    else {
      calc_crc = (calc_crc << 1);  
    }  
  }  
  calc_crc ^= (llsb); // do lsb
  for (bitmask = 8; bitmask > 0; --bitmask)  {
    if (calc_crc & 0x80) {
      calc_crc = (calc_crc << 1) ^ 0x131; // polynomial from Table 17
    }
    else {
      calc_crc = (calc_crc << 1);
    }  
  }  
  #ifdef DEBUG
    Serial.println("------------------");
    Serial.print("msb: " + String(lmsb,HEX) + " lsb: " + String(llsb,HEX));
    Serial.println(" crc: " + String(lcrc,HEX) + ""  calculated checksum = " + String(calc_crc,HEX));
    Serial.println("------------------");
  #endif
  if (calc_crc != lcrc) {
    return false;     // checksum error
  }
  else {
    return true;      // no error
  }
}
