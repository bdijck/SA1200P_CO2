# SA1200P_CO2
My spin on smartification of SA1200P CO2 meter (big thanks to weigu.lu)

Basically, we have a SA1200P CO2 meter, with an added ESP8266 to 'smartify' the device.

The ESP8266 gets the CO2 from the SA1200P.

Temperature, humidity and pressure are taken from an added BME280 sensor that is also connected to the ESP8266.

Note: ESP8266 uses fixed WIFI-settings (SSID "DijckNoyens_24G"). If this ssid or password would change, code needs to be updated. (Updated firmware can be uploaded using a temporary WIFI SSID/PWD that uses the old setting).

Details: see code

# How to use

http://192.168.1.156/update --> to update firmware over WIFI

http://192.168.1.156/data --> show CO2, temperature, humidity and pressure

Of course, IP address can change.