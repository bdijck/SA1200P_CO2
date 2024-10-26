# SA1200P_CO2
My spin on smartification of SA1200P CO2 meter (big thanks to weigu.lu)

Basically, we have a SA1200P CO2 meter, with an **added** ESP8266 to 'smartify' the device.

The ESP8266 gets the CO2 from the SA1200P.

Temperature, humidity and pressure are taken from an **added** BME280 sensor that is also connected to the ESP8266.

Note: ESP8266 uses hardcoded WIFI-settings. This means: if the SSID or password would change, code needs to be updated. (Hint to self: to update the firmware after WiFi SSID/PWD changes, _temporary_ use hotspotold with old WIFI SSID/PWD).

Details: see code

# How to use

http://IP.OF.THE.DEVICE/update --> to update firmware over WIFI

http://IP.OF.THE.DEVICE/data --> show CO2, temperature, humidity and pressure
