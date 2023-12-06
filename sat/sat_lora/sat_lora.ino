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

Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

/*
* GPS szenzor könyvtár importálás
*/
 
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

EspSoftwareSerial::UART locSerial;
Adafruit_GPS GPS(&locSerial);

unsigned long delayTime;

#include <HardwareSerial.h>

//HardwareSerial loraSerial();

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

float pInit = 0;
float seaLevel;

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

/*
* Mikrokontroller initializáció
*/
sensors_event_t event; 

void setup() {
    locSerial.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, 7, 8);
    Serial0.begin(115200, SERIAL_8N1, 1, 0);
    Serial.begin(115200);
    delay(1000);
    
    //while(!Serial); // Várakozás a serial kapcsolat felépülésére
     
    /*
    * BME280 initializáció
    */
    
    //Serial.println("BME280 test");

    //unsigned status;

    //status = 
    bme.begin();  

    //if (!status) {
    //    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    //    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
    //    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    //    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    //    Serial.print("        ID of 0x60 represents a BME 280.\n");
    //    Serial.print("        ID of 0x61 represents a BME 680.\n");
    //    while (1) delay(10);
    //}

    /*
    * GPS initializáció
    */
    
    //Serial.println("GPS test");

    if(locSerial){

      GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
      GPS.sendCommand(PGCMD_ANTENNA);

      delay(1000);
  
      locSerial.println(PMTK_Q_RELEASE);
    } else {
      while (true){
        Serial.println("zsido");
      }
    }

    

    if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    //Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_2_G);

  delay(2000);

  for(int i = 0; i<20; i++){
    pInit = pInit + (bme.readPressure()/100.0F);
    accel.getEvent(&event);
    xInit = xInit + event.acceleration.x;
    yInit = yInit + event.acceleration.y;
    zInit = zInit + event.acceleration.z;
    delay(100);
  }

  xHiba = xVart-(xInit/20);
  yHiba = yVart-(yInit/20);
  zHiba = zVart-(zInit/20);
  seaLevel = pInit/20;
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

uint32_t timer = millis();

void loop() { 
    String gps;
    char c = GPS.read();

    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) return;
    }

    if (millis() - timer > 200) {
      timer = millis();
      //int sensorPin = A7;
      //int sensorValue = analogRead(sensorPin);
      //gps|gps_speed|gps_muhold|gps_time|gps_angle|temperature|humidity|pressure|calibrated_alt|x_real|y_real|z_real|x|y|z
      //if(Serial0.available()){
        Serial0.println("radio tx " + stringToHex(gpsRead() + "|" + bmeRead() + "|" + acceleroRead()) + " 1");
      //} else {
        //Serial.println("radio tx " + gpsRead() + "|" + bmeRead() + "|" + acceleroRead() + "|" + " 1");
        //Serial.println(Serial0.read());
      //}
    }
}

String stringToHex(String input) {
  String output = "";

  // A karakterek végigiterálása
  for (int i = 0; i < input.length(); i++) {
    // Karakter konvertálása hexadecimális formába
    char hexChar[3];
    sprintf(hexChar, "%02X", input[i]);

    // Hexadecimális karakterlánc hozzáadása az eredményhez
    output += hexChar;
  }

  return output;
}

String acceleroRead() {
  sensors_event_t event; 
  accel.getEvent(&event);
  return String(event.acceleration.x) + "|" + String(event.acceleration.y) + "|" + String(event.acceleration.z)  + "|" + String(event.acceleration.x + xHiba) + "|" + String(event.acceleration.y + yHiba) + "|" + String(event.acceleration.z + zHiba);
}

String bmeRead() {
  return String(bme.readTemperature()) + "|" + String(bme.readHumidity()) + "|" + String(bme.readPressure() / 100.0F)  + "|" + String(bme.readAltitude(seaLevel));
}

String gpsRead() {
  String gpsdecimal, mgpsdecimal, sgpsdecimal;

  if (GPS.hour < 10) { gpsdecimal = "0"; }
  if (GPS.minute < 10) { mgpsdecimal = "0"; }
  if (GPS.seconds < 10) { sgpsdecimal = "0"; }

  return String(GPS.latitudeDegrees, 5) + ", " + String(GPS.longitudeDegrees, 5) + "|" + String(GPS.satellites) + "|20" + String(GPS.year) + "-" + String(GPS.month) + "-" + String(GPS.day) + " " + String(gpsdecimal) + String(GPS.hour) + ":" + String(mgpsdecimal) + String(GPS.minute) + ":" + String(sgpsdecimal) + String(GPS.seconds) + "|" + String(GPS.angle);
}
