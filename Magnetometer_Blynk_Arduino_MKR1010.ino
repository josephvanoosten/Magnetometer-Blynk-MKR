/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial
#include "arduino_secrets.h"
#include <SPI.h>
#include <WiFiNINA.h>
#include <BlynkSimpleWiFiNINA.h>
#include <Wire.h>
#include <SD.h>                                             // SD v1.2.3 for use of SD cart
#include <RTCZero.h>                                        // RTCZero 1.6.0 for epoch

#include <Adafruit_GFX.h>                                       // Graphics library
#include <Adafruit_SSD1306.h>                                   // Display
#define OLED_RESET -1
#define SCREEN_WIDTH 128                                        // OLED display width, in pixels
#define SCREEN_HEIGHT 32                                        // OLED display height, in pixels
#define GRAPH_LENGTH 100 

File SDF;
RTCZero yvo;                                                // Create an rtc object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

char ssid[] = SECRET_SSID;                        // your network SSID (name)
char pass[] = SECRET_PASS;                        // your network password (use for WPA, or use as key for WEP)
char authA[] = SECRET_AUTHA;
char authB[] = SECRET_AUTHB;
char authC[] = SECRET_AUTHC;
char authD[] = SECRET_AUTHD;
char authE[] = SECRET_AUTHE;
char authF[] = SECRET_AUTHF;

#define CALIBRATION 2
#define LED 3                                               // for indicating calibration sactive

#define SAMPLES 1000                                        // keep 2000 measurements in Array (somewhat like 10 seconds)
#define TIMEINBETWEEN 4

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
String dataString;

long MxArray[SAMPLES + 1];
long* MxArray_ptr = MxArray;
long MyArray[SAMPLES + 1];
long* MyArray_ptr = MyArray;
long MzArray[SAMPLES + 1];
long* MzArray_ptr = MzArray;
int Mx, Mxtot = 0;
int My, Mytot = 0;
int Mz, Mztot = 0;

int runMeas = 55;                                         // time for taking measurements in s
int numMeas = 0;                                            // number of measurements taken 
int startMeas = 1;                                          // for first run

int calibration = 0;
const float Mr = 53198; // reference value value Kristine Moen Huset
int xc = 0; // correction standard value
int yc = -216;
int zc = -2;
float xm = 14.02; // multiplication standard value
float ym = 14.75;
float zm = 14.69;

WidgetBridge bridge1(V20);                                  //Initiating Bridge Widget on V10 of this Device
WidgetBridge bridge2(V21);                                  //Initiating Bridge Widget on V10 of this Device
WidgetBridge bridge3(V22);                                  //Initiating Bridge Widget on V10 of this Device
WidgetBridge bridge4(V23);                                  //Initiating Bridge Widget on V10 of this Device

void setup()  {
  Wire.begin();                                             // Initialize serial and wait for port to open:
  Serial.begin(9600);
  Serial.println("wait for 5 seconds");
  Serial.println();
  delay(5000);                                            

  pinMode(LED, OUTPUT); 
  pinMode(CALIBRATION, INPUT_PULLUP);                       // red button button1 to start and stop calibration

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);                    // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();                                       // clear before new drawing
  display.display();                                            // display recent drawing
  Startscreen();
    
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
  
  if (!SD.begin(4)) {                                         // Initialize SD card reader
    Serial.println("SD Card initialization failed!");
    while (1);
  }
  else {
  Serial.println("SD Card found");
  }

  Blynk.begin(authE, ssid, pass);
  
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
}

void loop()   {
  Blynk.run();  
  lastConnectionTime = millis();
  numMeas = 0;
  long runTime = 100 * runMeas;
  do  {
    measure();
    numMeas = numMeas + 1;
  }   while (((millis() - lastConnectionTime) < (runTime)));
  Mx = ((Mxtot  * xm) / SAMPLES) - xc;
  My = ((Mytot  * ym) / SAMPLES) - yc;
  Mz = ((Mztot  * zm) / SAMPLES) - zc;

  dataString = getTimeStamp();
   
  write_SDdata(); 
    
    Blynk.virtualWrite(V2, Mx);
    Blynk.virtualWrite(V3, My);
    Blynk.virtualWrite(V4, Mz);

    bridge1.virtualWrite(V20, Mx, My, Mz);
    bridge2.virtualWrite(V21, Mx, My, Mz);
    bridge3.virtualWrite(V22, Mx, My, Mz);
    bridge4.virtualWrite(V23, Mx, My, Mz);
  
  int last = (millis() - lastConnectionTime);
  showValues(last, Mx, My, Mz, numMeas);
  
  if (calibration > 0) calibrate();
}

void measure() { 
  writeMag(QRM3100_POLL, 0x70);                             // poll the RM3100 for a single XYZ measurement
//  writeMag(QRM3100_POLL, 0x30);                           // poll the RM3100 for a single XY measurement
//  writeMag(QRM3100_POLL, 0x10);                           // poll the RM3100 for a single X measurement
  delay(TIMEINBETWEEN);                                                 // delay to help monitor DRDY pin on eval board
  while ((readMag(QRM3100_STATUS) & 0x80) != 0x80) {}       // Check if DRDY went high and wait unit high before reading results
    MxArray_ptr[0] =(((readMag(QRM3100_MX2)<<24) | (readMag(QRM3100_MX1) <<16) | readMag(QRM3100_MX0)<<8)>>8);
    MyArray_ptr[0] =(((readMag(QRM3100_MY2)<<24) | (readMag(QRM3100_MY1) <<16) | readMag(QRM3100_MY0)<<8)>>8);
    MzArray_ptr[0] =(((readMag(QRM3100_MZ2)<<24) | (readMag(QRM3100_MZ1) <<16) | readMag(QRM3100_MZ0)<<8)>>8);
    if (startMeas > 0) {
      for (int k = 1; k <= SAMPLES; k ++) {
        MxArray_ptr[k] = MxArray_ptr[0];
        MyArray_ptr[k] = MyArray_ptr[0];
        MzArray_ptr[k] = MzArray_ptr[0];
      }
      Mxtot = SAMPLES * MxArray_ptr[0];
      Mytot = SAMPLES * MyArray_ptr[0];
      Mztot = SAMPLES * MzArray_ptr[0];
      startMeas = -1;
    }
    Mxtot = Mxtot + (MxArray_ptr[0] - MxArray_ptr[SAMPLES]);
    Mytot = Mytot + (MyArray_ptr[0] - MyArray_ptr[SAMPLES]);
    Mztot = Mztot + (MzArray_ptr[0] - MzArray_ptr[SAMPLES]);    
    for (int i = (SAMPLES-1); i >= 0; i --) {      
      MxArray_ptr[(i + 1)] = MxArray_ptr[i];
      MyArray_ptr[(i + 1)] = MyArray_ptr[i];
      MzArray_ptr[(i + 1)] = MzArray_ptr[i];   
    }
}

void showValues(int one, int two , int three, int four, int five)    {
    display.clearDisplay(); //clear before new drawing
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(4,0);
    display.print(one,DEC);
    display.setCursor(40,0);
    display.print("ms  / #");
    display.setCursor(90,0);
    display.print(five);
    display.setCursor(4,9);
    display.print("Mx:");
    display.setCursor(40,9);
    display.print(two,DEC);
    display.setCursor(90,9);
    display.print("nT");
    display.setCursor(4,17);
    display.print("My:");
    display.setCursor(40,17);
    display.print(three,DEC);
    display.setCursor(90,17);
    display.print("nT");
    display.setCursor(4,25);
    display.print("Mz:");
    display.setCursor(40,25);
    display.print(four,DEC);
    display.setCursor(90,25);
    display.print("nT");
    display.display();  //needed before anything is displayed
    
    Serial.print(one,DEC);                                       // Print results
    Serial.print("   ");
    Serial.print(two,DEC);
    Serial.print("   ");
    Serial.print(three,DEC);
    Serial.print("   ");
    Serial.print(four,DEC);
    Serial.print("   ");
    Serial.print(five,DEC);
    Serial.println();
    
}

void calibrate() {
  Serial.println("Calibrate");
  long x = 0;
  long y = 0;
  long z = 0;
  long xl = 0;
  long xh = 0;
  long yl = 0;
  long yh = 0;
  long zl = 0;
  long zh = 0;
  while (calibration > 0) {
    Blynk.run();
    writeMag(QRM3100_POLL, 0x70);                             // poll the RM3100 for a single XYZ measurement
    delay(4);                                                 // delay to help monitor DRDY pin on eval board
    while ((readMag(QRM3100_STATUS) & 0x80) != 0x80) {}       // Check if DRDY went high and wait unit high before reading results
    x = long(((readMag(QRM3100_MX2)<<24) | (readMag(QRM3100_MX1) <<16) | readMag(QRM3100_MX0)<<8)>>8);
    y = long(((readMag(QRM3100_MY2)<<24) | (readMag(QRM3100_MY1) <<16) | readMag(QRM3100_MY0)<<8)>>8);
    z = long(((readMag(QRM3100_MZ2)<<24) | (readMag(QRM3100_MZ1) <<16) | readMag(QRM3100_MZ0)<<8)>>8); 
    xh = max(xh, x);
    xl = min(xl, x);
    yh = max(yh, y);
    yl = min(yl, y);
    zh = max(zh, z);
    zl = min(zl, z);
    xc = int((xh + xl)/2 + 0.5);
    yc = int((yh + yl)/2 + 0.5);
    zc = int((zh + zl)/2 + 0.5);
    xm = 2*Mr/(xh - xl);
    ym = 2*Mr/(yh - yl);
    zm = 2*Mr/(zh - zl);

    delay(100);

    Serial.print(xc, DEC);
    Serial.print("\t");
    Serial.print(yc, DEC);
    Serial.print("\t");
    Serial.print(zc, DEC);
    Serial.print("\t");
    Serial.print(xm, 2);
    Serial.print("\t");
    Serial.print(ym, 2);
    Serial.print("\t");
    Serial.println(zm, 2);

    display.clearDisplay(); //clear before new drawing
    display.display();  //needed before anything is displayed
    display.setCursor(4,2);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print("X      Y       Z");
    display.setCursor(4,12);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print(xl,DEC);
    display.setCursor(46,12);
    display.print(yl,DEC);
    display.setCursor(88,12);
    display.print(zl,DEC);
    display.setCursor(4,24);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print(xh,DEC);
    display.setCursor(46,24);
    display.print(yh,DEC);
    display.setCursor(88,24);
    display.print(zh,DEC);
    display.display();  //needed before anything is displayed
  }
}

BLYNK_WRITE(V11) {   
  calibration = param.asInt(); // Get value as integer
  Serial.print(calibration);
  Serial.println("  start/stop kalibreren");
}

BLYNK_WRITE(V12) {   
  runMeas = param.asInt(); // Get value as integer
  Serial.print(runMeas);
  Serial.println("  change measurement time");
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

void write_SDdata() {                                       // Write the data to the SD card
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
    SDF.print(Mx);
    SDF.print(separator);
    SDF.print(My);
    SDF.print(separator);
    SDF.print(Mz);
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

void Startscreen() {
  display.clearDisplay(); //clear before new drawing
  display.setCursor(4,2);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.print("Magnetometer");
  display.setCursor(4,20);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.print("Starting.....");
  display.display();  //needed before anything is displayed
}
