/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial
#include "arduino_secrets.h"
#include <SPI.h>
#include <WiFiNINA.h>
#include <BlynkSimpleWiFiNINA.h>
#include <Wire.h>
#include <Statistic.h>                                      // for cancelling noise
#include <SD.h>                                             // SD v1.2.3 for use of SD cart
#include <RTCZero.h>                                        // RTCZero 1.6.0 for epoch

File SDF;
Statistic Xstats;                                           // for measurement
RTCZero yvo;                                                // Create an rtc object

char ssid[] = SECRET_SSID;                        // your network SSID (name)
char pass[] = SECRET_PASS;                        // your network password (use for WPA, or use as key for WEP)
char authA[] = SECRET_AUTHA;
char authB[] = SECRET_AUTHB;
char authC[] = SECRET_AUTHC;
char authD[] = SECRET_AUTHD;
char authE[] = SECRET_AUTHE;
char authF[] = SECRET_AUTHF;

#define DEG_PER_RAD (180.0/3.14159265358979)
#define CALIBRATION 2
#define LED 3
#define SAMPLE1 1000
#define TIMEBETWEEN1 4
#define SAMPLE2 5
#define QRM3100_POLL      0x00                              //poll mode
#define QRM3100_CMM       0x01                              //continous measurement mode
#define QRM3100_TMRC      0x0B                              //setting update rate
#define QRM3100_I2C_ADDRESS     0x20                    
#define QRM3100_MX2       0x24                              // measurement results for x-axis
#define QRM3100_MX1       0x25
#define QRM3100_MX0       0x26
#define QRM3100_MY2       0x27                              // measurement results for y-axis
#define QRM3100_MY1       0x28
#define QRM3100_MY0       0x29
#define QRM3100_MZ2       0x2A                              // measurement results for z-axis
#define QRM3100_MZ1       0x2B
#define QRM3100_MZ0       0x2C
#define QRM3100_STATUS    0x34                              //status of DRDY
#define QRM3100_REVID     0x36                              //Address of slave device

unsigned long epoch;
int numberOfTries = 0, maxTries = 6;
const int GMT = 0;                                          //change this to adapt it to your time zone

unsigned long lastConnectionTime = 0;                       // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 6L * 1000L;           // delay between updates, in milliseconds

char fname[] = "AURDATA.txt";                               // data log file Name
char server[] = "www.dweet.io";
String dataString;

bool useStdDev = true;
bool notuseStdDev = false;

double ArrayX[SAMPLE2+1];                                   // for (moving) stddev of last  measurements
int xstdev = 0;
int strt = 0;

int calibration = 0;
const int Mr = 53198; // reference value value Kristine Moen Huset
int xc = 0; // correction standard value
int yc = 0;
int zc = 0;
int xm = 13; // multiplication standard value
int ym = 13;
int zm = 13;

WidgetBridge bridge1(V20);                                  //Initiating Bridge Widget on V10 of this Device
WidgetBridge bridge2(V21);                                  //Initiating Bridge Widget on V10 of this Device
WidgetBridge bridge3(V22);                                  //Initiating Bridge Widget on V10 of this Device
WidgetBridge bridge4(V23);                                  //Initiating Bridge Widget on V10 of this Device

BlynkTimer timer;                                           // Create a Timer object called "timer"! 


void setup()  {
  Wire.begin();                                             // Initialize serial and wait for port to open:
  Serial.begin(9600);
  Serial.println("wait for 5 seconds");
  Serial.println();
  delay(5000);                                            

  pinMode(LED, OUTPUT); 
  pinMode(CALIBRATION, INPUT_PULLUP);                       // red button button1 to start and stop calibration
  
  if (readMag(QRM3100_REVID) != 0x22) {                     // Check REVID register first, should return 0x22.
    Serial.println("REVID NOT CORRECT!");
    Serial.print("REVID: ");
    Serial.println(readMag(QRM3100_REVID), HEX);
  }
  else  {
    Serial.println("RM3100 Detected Properly");
    Serial.print("REVID: ");
    Serial.println(readMag(QRM3100_REVID), HEX);
  }
  Serial.print("Using Standard Calibration:  ");
  Serial.println(xc, 2);
  
  if (!SD.begin(4)) {                                         // Initialize SD card reader
    Serial.println("SD Card initialization failed!");
    while (1);
  }
  else {
  Serial.println("SD Card found");
  }

  Blynk.begin(authE, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  
  delay(10000);
  
  yvo.begin();                                                // Initialize RTC
  do {
    epoch = WiFi.getTime();
    numberOfTries++;
  }
  while ((epoch == 0) && (numberOfTries < maxTries));
  if (numberOfTries == maxTries) {
    Serial.print("NTP unreachable!!");
    while (1);
  }
  else {
    epoch = epoch + (GMT * 3600UL);                           // adjust local time  
    yvo.setEpoch(epoch);
    Serial.print("Epoch received: ");
    printP02D(yvo.getHours());
    Serial.print(":");
    printP02D(yvo.getMinutes());
    Serial.print(":");
    printP02D(yvo.getSeconds());
    Serial.print(" ");
    Serial.print(yvo.getDay());
    Serial.print("/");
    Serial.print(yvo.getMonth());
    Serial.print("/");
    Serial.print(yvo.getYear());
    Serial.println();
  }

  timer.setInterval(6000L, measure1);                         //  Here you set interval (msec) and which function to call 

}

void loop()   {
  Blynk.run();  
  timer.run();
}

void measure1() {
    int Mx;                                                     // variables to store results
    float xstddev;
    int R = 1000;                                                // Sample size  
    
    Xstats.clear(useStdDev);
  
    while (Xstats.count() < R) {
//      writeMag(QRM3100_POLL, 0x70);                             // poll the RM3100 for a single XYZ measurement
//      writeMag(QRM3100_POLL, 0x30);                           // poll the RM3100 for a single XY measurement
      writeMag(QRM3100_POLL, 0x10);                           // poll the RM3100 for a single X measurement
      delay(4);                                                 // delay to help monitor DRDY pin on eval board
      while ((readMag(QRM3100_STATUS) & 0x80) != 0x80) {}       // Check if DRDY went high and wait unit high before reading results
  
      Xstats.add((((readMag(QRM3100_MX2) * 65536) + (readMag(QRM3100_MX1) * 256) + readMag(QRM3100_MX0))<<8)>>8);
    }
    Mx = (Xstats.average()- xc)*xm;

    xstddev = Xstats.pop_stdev()*xm;
    
    if (strt == 0) {
      for(int i = 0; i < (SAMPLE2+1) ; i++) {                          //initial filling of the array
        ArrayX[i] = Mx;
        strt = 1;
      }
    } 
    ArrayX[0] = Mx;
    double sumXsquared = 0;
    double sumX = 0;
    for(int i = (SAMPLE2 - 1); i >= 0; i--){
      sumXsquared += pow(ArrayX[i],2);
      sumX += ArrayX[i];
      ArrayX[i+1] = ArrayX[i];
    }
    xstdev = int((sumXsquared - pow(sumX,2)/SAMPLE2)/(SAMPLE2 - 1));
    
    int last = (millis() - lastConnectionTime);

    
    dataString = getTimeStamp(); 
    write_SDdata(Mx); 
    
    Serial.print(last,DEC);                                       // Print results
    Serial.print("   ");
    Serial.print(xstddev,2);
    Serial.print("   ");
    Serial.print(xstdev,DEC);
    Serial.print("   ");
    Serial.print(Mx,DEC);
    Serial.println();
    
    Blynk.virtualWrite(V2, Mx);
    Blynk.virtualWrite(V3, xstdev);

    bridge1.virtualWrite(V20, Mx, xstdev);
    bridge2.virtualWrite(V21, Mx, xstdev);
    bridge3.virtualWrite(V22, Mx, xstdev);
    bridge4.virtualWrite(V23, Mx, xstdev);
    
    
    lastConnectionTime = millis();
}


void calibrate() {
  Serial.println("Calibrate");
  int x = 0;
  int y = 0;
  int z = 0;
  int xl = 0;
  int xh = 0;
  int yl = 0;
  int yh = 0;
  int zl = 0;
  int zh = 0;
  while (calibration > 0) {
    Blynk.run();
    writeMag(QRM3100_POLL, 0x70);                             // poll the RM3100 for a single XYZ measurement
    delay(4);                                                 // delay to help monitor DRDY pin on eval board
    while ((readMag(QRM3100_STATUS) & 0x80) != 0x80) {}       // Check if DRDY went high and wait unit high before reading results
    x = int((((readMag(QRM3100_MX2) * 65536) + (readMag(QRM3100_MX1) * 256) + readMag(QRM3100_MX0))<<8)>>8);
    y = int((((readMag(QRM3100_MY2) * 65536) + (readMag(QRM3100_MY1) * 256) + readMag(QRM3100_MY0))<<8)>>8);
    z = int((((readMag(QRM3100_MZ2) * 65536) + (readMag(QRM3100_MZ1) * 256) + readMag(QRM3100_MZ0))<<8)>>8); 
    xh = max(xh, x);
    xl = min(xl, x);
    yh = max(yh, y);
    yl = min(yl, y);
    zh = max(zh, z);
    zl = min(zl, z);
    xc = int((xh + xl)/2 + 0.5);
    yc = int((yh + yl)/2 + 0.5);
    zc = int((zh + zl)/2 + 0.5);
    xm = int(2*Mr/(xh - xl) + 0.5);
    ym = int(2*Mr/(yh - yl) + 0.5);
    zm = int(2*Mr/(zh - zl) + 0.5);

    delay(100);

    Serial.print(xc, DEC);
    Serial.print("\t");
    Serial.print(yc, DEC);
    Serial.print("\t");
    Serial.print(zc, DEC);
    Serial.print("\t");
    Serial.print(xm, DEC);
    Serial.print("\t");
    Serial.print(ym, DEC);
    Serial.print("\t");
    Serial.println(zm, DEC);
  }
}

BLYNK_WRITE(V4) {   
  calibration = param.asInt(); // Get value as integer
  Serial.print(calibration);
  Serial.println("  start/stop kalibreren");
}

BLYNK_CONNECTED() {
  bridge1.setAuthToken(authB); // Token Aurora Alert1
  bridge2.setAuthToken(authC); // Token Yun
  bridge3.setAuthToken(authD); // Token M5Stack
  bridge4.setAuthToken(authF); // Token Aurora Alert2

}


uint8_t readMag(uint8_t address) {
  uint8_t output;
  Wire.beginTransmission(QRM3100_I2C_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  delayMicroseconds(2);
  Wire.requestFrom(QRM3100_I2C_ADDRESS, 1);
  while (Wire.available())  {
    output = Wire.read();
  }
  return output;
}

void writeMag(uint8_t address, uint8_t value) {
  Wire.beginTransmission(QRM3100_I2C_ADDRESS);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}

void printP02D(int number) {
  if (number < 10) Serial.print('0');
  Serial.print(number);
}

void write_SDdata(int M) {                                       // Write the data to the SD card
  String separator = ", ";                                  // Data Line as follow (with comma separator):
  SDF = SD.open(fname, FILE_WRITE);
  if (!SDF) {                                               // terminal error if we can't open the SD File (we already initialized)
    Serial.println("SD card write failure!");
    while (1);
  }
  else {
    SDF.print(yvo.getDay());
    SDF.print("/");
    SDF.print(yvo.getMonth());
    SDF.print("/");
    SDF.print(yvo.getYear());
    SDF.print("-");
    SDF.print(yvo.getHours());
    SDF.print(":");
    SDF.print(yvo.getMinutes());
    SDF.print(":");
    SDF.print(yvo.getSeconds());
    SDF.print(separator);
    SDF.print(M);
    SDF.print(separator);
    SDF.print(xstdev);
    SDF.println();  // Windows cr/lf
    SDF.close();
  }
}

String getTimeStamp() {
   String result;
   result=(String(yvo.getDay())+"/"+String(yvo.getMonth())+"/"+String(yvo.getYear())+"-"+String(yvo.getHours())+":"+String(yvo.getMinutes())+":"+String(yvo.getSeconds()));
   return result;
}

String Dubbel(int number) {
  String Twee = "0";
  if (number > 9) Twee += String(number);
  Twee = String(number);
}
