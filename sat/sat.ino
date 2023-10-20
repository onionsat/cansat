/*
* Hőmérséklet+páratartalom+légnyomás (BME280) szenzor könyvtár importálás
*/
 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK 12
#define BME_MISO 11
#define BME_MOSI 10
#define BME_CS 9

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

/*
* GPS szenzor könyvtár importálás
*/
 
#include <Adafruit_GPS.h>
#include <HardwareSerial.h>

#define GPSSerial Serial0

Adafruit_GPS GPS(&GPSSerial);

unsigned long delayTime;

/*
* ADXL345 initializáció
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define LED_BUILTIN 13

float xInit = 0;
float yInit = 0;
float zInit = 0;

float xHiba;
float yHiba;
float zHiba;

float xVart = 0;
float yVart = 0;
float zVart = 9.81;


/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

/*
* Mikrokontroller initializáció
*/
sensors_event_t event; 

void setup() {
    Serial.begin(250000);
    //while(!Serial); // Várakozás a serial kapcsolat felépülésére
    delay(5000);
     
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
  
    GPSSerial.println(PMTK_Q_RELEASE);

    if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_2_G);

  delay(2000);

  for(int i = 0; i<20; i++){
    accel.getEvent(&event);
    xInit = xInit + event.acceleration.x;
    yInit = yInit + event.acceleration.y;
    zInit = zInit + event.acceleration.z;
    delay(100);
  }

  xHiba = xVart-(xInit/20);
  yHiba = yVart-(yInit/20);
  zHiba = zVart-(zInit/20);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

uint32_t timer = millis();

void loop() { 
    char c = GPS.read();

    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) return;
    }

    if (millis() - timer > 100) {
      timer = millis();
      
      Serial.println("--------- BME280 adat ---------");
      bmeRead();
      Serial.println("--------- GPS adat ---------");
      gpsRead();
      Serial.println("--------- ADXL adat ---------");
      acceleroRead();
    }
}

void acceleroRead() {
  sensors_event_t event; 
  accel.getEvent(&event);
  Serial.print("X (valos): "); Serial.println((event.acceleration.x));
  Serial.print("Y (valos): "); Serial.println((event.acceleration.y));
 Serial.print("Z (valos): "); Serial.println((event.acceleration.z));
  /* Display the results (acceleration is measured in m/s^2) */
 Serial.print("X: "); Serial.println((event.acceleration.x + xHiba));
 Serial.print("Y: "); Serial.println((event.acceleration.y + yHiba));
 Serial.print("Z: "); Serial.println((event.acceleration.z + zHiba));
}

void bmeRead() {
    Serial.print("Homerseklet: "); Serial.print(bme.readTemperature()); Serial.println(" °C");
    Serial.print("Legnyomas: "); Serial.print(bme.readPressure() / 100.0F); Serial.println(" hPa");
    Serial.print("Tszf. magassag (kb) = "); Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA)); Serial.println(" m");
    Serial.print("Paratartalom: "); Serial.print(bme.readHumidity()); Serial.println(" %");
}

void gpsRead() {
    Serial.print("Fix muholdak: "); Serial.print((int)GPS.fix); Serial.print(" (minoseg: "); Serial.print((int)GPS.fixquality); Serial.println(")");
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
