/*
* SD kártya inicializáció
*/

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "TimeLib.h"

unsigned long unix_timestamp;  // a timestamp
unsigned int timestamp;  // a timestamp

void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.path(), levels - 1);
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

void createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

String readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);
  String adat = "ßß-NA$$-";
  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return "ßß-NA$$-";
  }

  adat = file.readString();
  file.close();
  return adat;
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

int nosave = 0;

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    nosave = 1;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
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
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}

String lora_mod, lora_sf, lora_pa, lora_crc, lora_cr, lora_freq, lora_pwr, lora_bw, lora_sync;

void extractDataFromString(String data) {
  // A String-et sorokra bontja a sortörés alapján
  String lines[6];  // A sorokat tároló tömb
  int index = 0;
  int startPos = 0;

  while (index < 6 && startPos < data.length()) {
    int endPos = data.indexOf('\n', startPos);
    if (endPos == -1) {
      endPos = data.length();  // Az utolsó sor
    }

    lines[index] = data.substring(startPos, endPos);
    startPos = endPos + 1;  // Az új sor kezdete
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
        lora_freq = value;
      } else if (key.equals("lora_pwr")) {
        lora_pwr = value;
      } else if (key.equals("lora_sf")) {
        lora_sf = value;
      } else if (key.equals("lora_bw")) {
        lora_bw = value;
      } else if (key.equals("lora_sync")) {
        lora_sync = value;
      } else if (key.equals("lora_pa")) {
        lora_pa = value;
      } else if (key.equals("lora_crc")) {
        lora_crc = value;
      } else if (key.equals("lora_cr")) {
        lora_cr = value;
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

Adafruit_BME280 bme;  // I2C

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

int flights = 1;


void setup() {

  locSerial.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, 7, 8);
  Serial0.begin(115200, SERIAL_8N1, 1, 0);
  Serial.begin(115200);
  delay(3000);

  Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    //while (1);
  }

  String settingsfiledata = readFile(SD, "/settings.csat");
  String flightcounter = readFile(SD, "/flightcounter.csat");

  createDir(SD, "/flight_logs");
  if (flightcounter == "ßß-NA$$-") {
    writeFile(SD, "/flightcounter.csat", "1");
  } else {
    flights = flightcounter.toInt();
    int nfew = flights + 1;
    writeFile(SD, "/flightcounter.csat", String(nfew).c_str());
  }

  //writeFile(SD, String("/flight_logs/"+String(flights)+".txt").c_str(), "");

  Serial0.println("sys reset");


  if (settingsfiledata == "ßß-NA$$-") {
    writeFile(SD, "/settings.csat", "lora_mod=lora\nlora_freq=868100000\nlora_pwr=1\nlora_sf=sf7\nlora_crc=on\nlora_cr=3/4\nlora_bw=125\nlora_sync=34\nlora_pa=on");
  } else {
    extractDataFromString(settingsfiledata);
    for (int i = 0; i < 5; i++) {
      Serial.println(i);
      delay(50);
      Serial0.println("radio set freq " + lora_freq);
      delay(50);
      Serial0.println("radio set bw 250");
      delay(50);
      Serial0.println("radio set pa " + lora_pa);
      delay(50);
      Serial0.println("radio set pwr " + lora_pwr);
      delay(50);
      Serial0.println("radio set sync " + lora_sync);
      delay(50);
      Serial0.println("radio set sf " + lora_sf);
      delay(50);
    }
  }
  
  /*Serial0.println("radio set freq 868200000");
  Serial0.println("radio set bw 125");
  Serial0.println("radio set pa off");
  Serial0.println("radio set pwr 7");
  Serial0.println("radio set sync 35");*/

  

  //while(!Serial); // Várakozás a serial kapcsolat felépülésére

  /*
    * BME280 initializáció
    */

  //Serial.println("BME280 test");

  //unsigned status;

  //status =
  bme.begin();
  float meres = 0;
  for (int i = 0; i < 5; i++){
    meres = meres + bme.readPressure() / 100.0F;
  }

  seaLevel = meres / 5;
  Serial.println(seaLevel);

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

  if (locSerial) {
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    //GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
    locSerial.println(PMTK_Q_RELEASE);
  } else {
    while (true) {
      Serial.println("zsido");
    }
  }

  if (!accel.begin()) {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1);
  }
}

uint32_t timer = millis();


int lcounter = 1;
int packets = 0;
void loop() {
  

  String gps;
  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  if (millis() - timer > 200) {
      packets++;

      if(lcounter == 1){
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_GREEN, LOW);

      lcounter = 2;
  } else if(lcounter == 2){
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_BLUE, HIGH);
      digitalWrite(LED_GREEN, LOW);    
      lcounter = 3;
  } else if(lcounter == 3){
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_GREEN, HIGH);
            lcounter = 1;
  }
    timer = millis();

    //gps|gps_muhold|gps_time|gps_angle|temperature|humidity|pressure|calibrated_alt|x_real|y_real|z_real|methane|co2

    String adat = gpsRead() + "|" + bmeRead() + "|" + acceleroRead();
    Serial0.println("radio tx " + stringToHex(adat) + " 1");
    Serial.println(adat);
    if(nosave == 0){
      int year = GPS.year + 2000;
      int month = GPS.month;
      int day = GPS.day;
      int hour = GPS.hour;
      int minute = GPS.minute;
      int second = GPS.seconds;
      if(year > 2000 && month > 0 && day > 0){
          timestamp = (int) getUnixTimestamp(year, month, day, hour, minute, second);
      } else {
        timestamp = packets*1000;
      }

      String adatsor = String(String(timestamp) + "000@#" + adat + "\n");
      appendFile(SD, String("/flight_logs/" + String(flights) + ".txt").c_str(), adatsor.c_str());
    }
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

unsigned long getUnixTimestamp(int year, int month, int day, int hour, int minute, int second) {
    const int secondsPerMinute = 60;
    const int secondsPerHour = 3600;
    const int secondsPerDay = 86400;
    const int daysInNonLeapYear = 365;
    const int daysInLeapYear = 366;
    
    int daysPerMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    
    // Számoljuk az évek eltelt napjait (az 1970 és a megadott év között)
    unsigned long days = (year - 1970) * daysInNonLeapYear;
    // Számoljuk a kerek éveket, amelyek a 4-gyel oszthatók (kivéve a 100-zal oszthatókat, kivéve a 400-zal oszthatókat)
    for (int y = 1970; y < year; ++y) {
        if ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0)) {
            days++; // szökőévek
        }
    }
    
    // Adjuk hozzá a hónapok napjait az eltelt naphoz
    for (int m = 0; m < month - 1; ++m) {
        days += daysPerMonth[m];
        // ha szökőév február, akkor növeljük a napok számát
        if (m == 1 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
            days++;
        }
    }
    
    // Adjuk hozzá a napokat
    days += day - 1;
    
    // Számoljuk az órák, percek és másodpercek eltelt másodperceit
    unsigned long timestamp = days * secondsPerDay + hour * secondsPerHour + minute * secondsPerMinute + second;
    
    return timestamp;
}

String acceleroRead() {
  sensors_event_t event;
  accel.getEvent(&event);
  return String(event.acceleration.x) + "|" + String(event.acceleration.y) + "|" + String(event.acceleration.z);
}

String bmeRead() {
  //temp|humidity|pressure|altitude
  return String(bme.readTemperature()) + "|" + String(bme.readHumidity()) + "|" + String(bme.readPressure() / 100.0F) + "|" + String(bme.readAltitude(seaLevel));
}

String gpsRead() {
  String gpsdecimal, mgpsdecimal, sgpsdecimal;

  if (GPS.hour < 10) { gpsdecimal = "0"; }
  if (GPS.minute < 10) { mgpsdecimal = "0"; }
  if (GPS.seconds < 10) { sgpsdecimal = "0"; }
  //coords|sat|date|angle

  return String(GPS.latitudeDegrees, 5) + ", " + String(GPS.longitudeDegrees, 5) + "|" + String(GPS.satellites) + "|20" + String(GPS.year) + "-" + String(GPS.month) + "-" + String(GPS.day) + " " + String(gpsdecimal) + String(GPS.hour) + ":" + String(mgpsdecimal) + String(GPS.minute) + ":" + String(sgpsdecimal) + String(GPS.seconds) + "|" + String(GPS.angle);
}