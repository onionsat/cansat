/*
* Hőmérséklet+páratartalom+légnyomás (BME280) szenzor könyvtár importálás
*/
 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

/*
* GPS szenzor könyvtár importálás
*/
 
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial gpsSerial(8, 7);
Adafruit_GPS GPS(&gpsSerial);
#define GPSECHO  true

unsigned long delayTime;

/*
* Mikrokontroller initializáció
*/

void setup() {
    Serial.begin(250000);
    while(!Serial); // Várakozás a serial kapcsolat felépülésére
    
    /*
    * BME280 initializáció
    */
    
    Serial.println("BME280 test");

    unsigned status;

    status = bme.begin();  

    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }

    /*
    * GPS initializáció
    */
    
    Serial.println("GPS test");

    GPS.begin(9600);

    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);

    delay(1000);
  
    gpsSerial.println(PMTK_Q_RELEASE);
}

uint32_t timer = millis();

void loop() { 
    char c = GPS.read();

    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) return;
    }

    if (millis() - timer > 500) {
      timer = millis();
      
      Serial.print("--------- BME280 adat ---------");
      bmeRead();
      Serial.print("--------- GPS adat ---------");
      gpsRead();
    }
}

void bmeRead() {
    Serial.print("Homerseklet: "); Serial.print(bme.readTemperature()); Serial.println(" °C");
    Serial.print("Legnyomas: "); Serial.print(bme.readPressure() / 100.0F); Serial.println(" hPa");
    Serial.print("Tszf. magassag (kb) = "); Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA)); Serial.println(" m");
    Serial.print("Paratartalom: "); Serial.print(bme.readHumidity()); Serial.println(" %");
}

void gpsRead() {
    Serial.print("Fix muholdak: "); Serial.print((int)GPS.fix); Serial.print(" (minoseg:"); Serial.println((int)GPS.fixquality); Serial.print(")");
    Serial.print("Datum: ");
    Serial.print("20"); Serial.print(GPS.year, DEC); Serial.print('-');
    Serial.print(GPS.month, DEC); Serial.print('-');
    Serial.println(GPS.day, DEC);
    
    Serial.print("Ido: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    
    if (GPS.fix) {
      Serial.print("Sebesseg (csomo): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Helyzet: "); Serial.print(GPS.latitude, 4); Serial.print(GPS.lat); Serial.print(", "); Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Koordinata: "); Serial.print(GPS.latitudeDegrees, 4); Serial.print(", "); Serial.println(GPS.longitudeDegrees, 4);
      Serial.print("Muholdak: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna statusz: "); Serial.println((int)GPS.antenna);
    }
}
