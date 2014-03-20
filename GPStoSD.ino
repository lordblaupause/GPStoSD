/*
 * GPStoSD
 * First Development Version for DIY Mobile Mapper Project.
 * GPS connected to digital Pin 3 (TX) and 4 (RX) and LCD Panel
 * connected to digital Pin 5 (TX) and 6 (RX). SD logging via 
 * SD shield using digital Pin 10.
 *
 * Postition is tracked periodically. Lon, Lat, Sat number, HDOP,
 * Date and Time is logged to SD card. Lon and Lat to LCD panel.
 *
 * This Sketch was made for development purposes only and will 
 * be licensed under CC.
 * GPS data handling adapted from TinyGPS example files.
 *
 * Author: Mirko Maelicke <mirko@maelicke-online.de>
 * visit: http://openhydro.org/typo3/index.php?id=38
 */
#include <SoftwareSerial.h>
#include <SerialLCD.h>
#include <TinyGPS.h>
#include <SPI.h>
#include <SD.h>

File fs;                      // filestream for logging file
SoftwareSerial gpsPort(3,4);  // serial port of GPS
TinyGPS gps;                  // gps object for parsing data
SerialLCD slcd(5,6);          // serial port of LCD

int sdPin = 10;               // SD pin
int interval = 1000;          // in milliseconds

void setup() {
  // wait for 0.1 sec
  delay(100);
  
  pinMode(sdPin, OUTPUT);   // Pin 10 is used for SD card
  if(!SD.begin(sdPin)){    // start SD card
    return;
  }
  
  // create log file header, if not exists
  if(!SD.exists("gps.txt")){
    fs = SD.open("gps.txt", FILE_WRITE);
    fs.println("GPS MAPPER OUTPUT FILE");
    fs.println("Sat,HDOP,Latitude,Longitude,Date,Time");
    fs.println("-------------------------------------");
    fs.close();
  }
  
  // start LCD panel
  slcd.begin();
  
  // start GPS device
  gpsPort.begin(9600);
}

void loop() {
  // newC indicates if new NMEA was collected
  bool newC = false;
  
  // collect NMEA string for 1 second
  for(unsigned long start = millis(); millis() - start < interval;) {
    while(gpsPort.available()) {
      char c = gpsPort.read();
      if (gps.encode(c))
        newC = true;    // NMEA found, so parse it.
    }
  }
  
  // clear LCD panel
  slcd.clear();
  
  // if GPS recieved new NMEA, parse it
  if(newC){
    float lat, lon;         // Position parameters
    unsigned long age;      // Coordinate age
    char clat[20];          // char buffer for latitude
    char clon[20];          // char buffer for longitude
    // parse position from NMEA
    gps.f_get_position(&lat, &lon, &age);
    
    // convert decimal degree into string
    dtostrf(double(lat), 3, 10, clat);
    dtostrf(double(lon), 3, 10, clon);
    // open logging file
    fs = SD.open("gps.txt", FILE_WRITE);
    
    // number of sattelites
    fs.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    fs.print(",");
    
    // quality value HDOP
    fs.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    fs.print(",");
    
    // Latitude as decimal degree
    fs.print(lat == TinyGPS::GPS_INVALID_F_ANGLE ? "NA" : clat);
    fs.print(",");
    slcd.setCursor(0,0);
    slcd.println(lat == TinyGPS::GPS_INVALID_F_ANGLE ? "LAT = N.A." : clat);
    
    // Longitude as decimal degree
    fs.println(lon == TinyGPS::GPS_INVALID_F_ANGLE ? "NA" : clon);
    fs.print(",");
    slcd.setCursor(0,1);
    slcd.print(lon == TinyGPS::GPS_INVALID_F_ANGLE ? "LON = N.A." : clat);
    
    // Date Time
    sdPrint_dateTime(fs, gps);

    fs.close();
  } else {
    // No NMEA recieved during last second
    // no Sattelite availale
    fs = SD.open("gps.txt", FILE_WRITE);
    fs.println("No Data recieved");
    
    // print to LCD
    slcd.setCursor(0,0);
    slcd.print("Sat. tracking");
    slcd.setCursor(0,1);
    slcd.print("not possible");
    fs.close();
  } 
}

/*
 *  Custom Function declaration
 */
static void sdPrint_dateTime(File &fs, TinyGPS &gps){
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    fs.print("NA");
  else {
    char buf[32];
    sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
        year, month, day, hour, minute, second);
    
    fs.print(buf);
  }
    
}
