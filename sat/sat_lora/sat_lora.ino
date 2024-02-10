/*
* SD kártya inicializáció
*/

#include "FS.h"
#include "SD.h"
#include "SPI.h"

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

String readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);
    String adat = "ßß-NA$$-";
    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return "ßß-NA$$-";
    }

    adat = file.readString();
    file.close();
    return adat;

}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

String lora_mod, lora_sf;
int lora_freq, lora_pwr, lora_bw, lora_sync;

void extractDataFromString(String data) {
  // A String-et sorokra bontja a sortörés alapján
  String lines[6]; // A sorokat tároló tömb
  int index = 0;
  int startPos = 0;
  
  while (index < 6 && startPos < data.length()) {
    int endPos = data.indexOf('\n', startPos);
    if (endPos == -1) {
      endPos = data.length(); // Az utolsó sor
    }

    lines[index] = data.substring(startPos, endPos);
    startPos = endPos + 1; // Az új sor kezdete
    index++;
  }

  // Sorokból az értékek kinyerése
  for (int i = 0; i < 6; i++) {
    int equalsIndex = lines[i].indexOf('=');
    if (equalsIndex != -1) {
      String key = lines[i].substring(0, equalsIndex);
      String value = lines[i].substring(equalsIndex + 1);

      if (key.equals("lora_mod")) {
        lora_mod = value;
      } else if (key.equals("lora_freq")) {
        lora_freq = value.toInt();
      } else if (key.equals("lora_pwr")) {
        lora_pwr = value.toInt();
      } else if (key.equals("lora_sf")) {
        lora_sf = value;
      } else if (key.equals("lora_bw")) {
        lora_bw = value.toInt();
      } else if (key.equals("lora_sync")) {
        lora_sync = value.toInt();
      }
    }
  }
}

/*
* Hőmérséklet+páratartalom+légnyomás (BME280) szenzor könyvtár importálás
*/
 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme; // I2C

/*
* GPS szenzor könyvtár importálás
*/
 
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

EspSoftwareSerial::UART carbondioxideSerial;
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
    carbondioxideSerial.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, 5, 6);
    Serial0.begin(115200, SERIAL_8N1, 1, 0);
    Serial.begin(115200);
    delay(5000);

    Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }

  String settingsfiledata = readFile(SD, "/settings.csat");

  if(settingsfiledata == "ßß-NA$$-"){
    writeFile(SD, "/settings.csat", "lora_mod=lora\nlora_freq=868100000\nlora_pwr=1\nlora_sf=sf7\nlora_bw=125\nlora_sync=34");
  }

  extractDataFromString(settingsfiledata);
  Serial0.println("radio set mod " + lora_mod);
  sleep(1);
  Serial0.println("radio set freq " + String(lora_freq));
  sleep(1);
  Serial0.println("radio set pwr " + String(lora_pwr));
  sleep(1);
  Serial0.println("radio set sf " + lora_sf);
  sleep(1);
  Serial0.println("radio set bw " + String(lora_bw));
  sleep(1);
  Serial0.println("radio set sync " + String(lora_sync));
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
}

uint32_t timer = millis();

void loop() { 

    String gps;
    char c = GPS.read();

    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) return;
    }

    if (millis() - timer > 50) {
      timer = millis();
      //int sensorPin = A7;
      //int sensorValue = analogRead(sensorPin);
      //gps|gps_speed|gps_muhold|gps_time|gps_angle|temperature|humidity|pressure|calibrated_alt|x_real|y_real|z_real|x|y|z
      //if(Serial0.available()){
        //Serial.println(carbondioxideSerial.read());
        Serial0.println("radio tx " + stringToHex(gpsRead() + "|" + bmeRead() + "|" + acceleroRead()) + " 1");
        Serial.println("radio tx " + stringToHex(gpsRead() + "|" + bmeRead() + "|" + acceleroRead()) + " 1");

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

String methaneRead() {
  return "0";
}
