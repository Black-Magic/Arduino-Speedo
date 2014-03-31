/* Tinyduino based GPS/Accelerometer speedometer
 http://www.tiny-circuits.com
Pin use:
GPS:            A0, A1, A2, A3        (Software serial)
Accelerometer:  A4, A5                (I2C A4=SDA. A5=SCL)
LED:            5, 6, 7, 8, 9         (I/O)
*/

//---- INCLUDES ----
//GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//Accelerometer
#include <Wire.h>

// Accelerometer
#define BMA250_I2CADDR      0x18
#define BMA250_RANGE        0x03   // 0x03 = 2g, 0x05 = 4g, 0x08 = 8g, 0x0C = 16g
#define BMA250_BW           0x08   // 7.81Hz (update time of 64ms)
int AccelX;
int AccelY;
int AccelZ;
float AccelTemperature;

// GPS
static const int GPS_ONOFFPin = A3;
static const int GPS_SYSONPin = A2;
static const int GPS_RXPin = A1;
static const int GPS_TXPin = A0;
static const int GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device (Soft serial port)
SoftwareSerial ss(GPS_RXPin, GPS_TXPin);

void setup()
{
  Serial.begin(115200);
  
//LED display
  pinMode(5, INPUT);  
  pinMode(6, INPUT);  
  pinMode(7, INPUT);  
  pinMode(8, INPUT);  
  pinMode(9, INPUT);
  
// Accelerometer
  Wire.begin();
  BMA250Init();
  
// GPS
  // Init the GPS Module to wake mode
  pinMode(GPS_SYSONPin, INPUT);
  pinMode(GPS_ONOFFPin, OUTPUT);
  digitalWrite( GPS_ONOFFPin, LOW );   
  delay(5); 
  if( digitalRead( GPS_SYSONPin ) == LOW )
  {
     // Need to wake the module
    digitalWrite( GPS_ONOFFPin, HIGH ); 
    delay(5); 
    digitalWrite( GPS_ONOFFPin, LOW );      
  } 
  
  ss.begin(GPSBaud);

  Serial.println(F("FullExample.ino"));
  Serial.println(F("An extensive example of many interesting TinyGPS++ features"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("---------------------------------------------------------------------------------------------------------------------------------------"));
}

void loop()
{
  //LEDs
  int byCount = 0; // byCount holds the LED to turn on (1-16)
  float LED_ind = 0;
  
  //Accel
  int AccelX_MAX = -500;
  int AccelX_MIN = 500;
  int AccelX_Range = 0;
  
  //GPS
  
  
  while(1)
  {
    //Accel
      BMA250ReadAccel();
      
      // Print out the accelerometer data
      Serial.print("x: ");
      Serial.print(AccelX);
      Serial.print(", y: ");
      Serial.print(AccelY);
      Serial.print(", z:");
      Serial.print(AccelZ);
      Serial.print(",  t: ");   
      Serial.print(AccelTemperature);
      Serial.print("degC");
      
      if ( AccelX_MAX < AccelX )
      {
        AccelX_MAX = AccelX;
      }
      if (AccelX_MIN > AccelX )
      {
        AccelX_MIN = AccelX;
      }
      AccelX_Range = AccelX_MAX - AccelX_MIN;
      
      Serial.print(",  led: ");
      LED_ind = 16 * (AccelX + (AccelX_Range/2)) / AccelX_Range;
      byCount = LED_ind;
      Serial.println(byCount);
    
    //LED
      LedOn( byCount );
    
    //GPS
      static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
    
      printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
      printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
      printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
      printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
      printInt(gps.location.age(), gps.location.isValid(), 5);
      printDateTime(gps.date, gps.time);
      printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
      printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
      printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
      printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);
    
      unsigned long distanceKmToLondon =
        (unsigned long)TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT, 
          LONDON_LON) / 1000;
      printInt(distanceKmToLondon, gps.location.isValid(), 9);
    
      double courseToLondon =
        TinyGPSPlus::courseTo(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT, 
          LONDON_LON);
    
      printFloat(courseToLondon, gps.location.isValid(), 7, 2);
    
      const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);
    
      printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);
    
      printInt(gps.charsProcessed(), true, 6);
      printInt(gps.sentencesWithFix(), true, 10);
      printInt(gps.failedChecksum(), true, 9);
      Serial.println();
      
      smartDelay(200);
    
      if (millis() > 5000 && gps.charsProcessed() < 10)
        Serial.println(F("No GPS data received: check wiring"));
      //END GPS 
    
    delay(10);
  }
}

//-------------------------------------------

void LedOn( int iLedNum )
{
   if( iLedNum == 1 )   
   {
      digitalWrite( 5, HIGH );
      digitalWrite( 6, LOW );
     
      pinMode(5, OUTPUT);  
      pinMode(6, OUTPUT);  
      pinMode(7, INPUT);  
      pinMode(8, INPUT);  
      pinMode(9, INPUT);        
   } 
   else if( iLedNum == 2 )   
   {
      digitalWrite( 5, LOW );
      digitalWrite( 6, HIGH );
     
      pinMode(5, OUTPUT);  
      pinMode(6, OUTPUT);  
      pinMode(7, INPUT);  
      pinMode(8, INPUT);  
      pinMode(9, INPUT);        
   } 
   else if( iLedNum == 3 )   
   {
      digitalWrite( 5, HIGH );
      digitalWrite( 7, LOW );
     
      pinMode(5, OUTPUT);  
      pinMode(6, INPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, INPUT);  
      pinMode(9, INPUT);        
   } 
   else if( iLedNum == 4 )   
   {
      digitalWrite( 7, HIGH );
      digitalWrite( 5, LOW );
     
      pinMode(5, OUTPUT);  
      pinMode(6, INPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, INPUT);  
      pinMode(9, INPUT);        
   }   
   else if( iLedNum == 5 )   
   {
      digitalWrite( 6, HIGH );
      digitalWrite( 7, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, OUTPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, INPUT);  
      pinMode(9, INPUT);        
   } 
   else if( iLedNum == 6 )   
   {
      digitalWrite( 7, HIGH );
      digitalWrite( 6, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, OUTPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, INPUT);  
      pinMode(9, INPUT);        
   }  
   else if( iLedNum == 7 )   
   {
      digitalWrite( 6, HIGH );
      digitalWrite( 8, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, OUTPUT);  
      pinMode(7, INPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, INPUT);        
   }    
   else if( iLedNum == 8 )   
   {
      digitalWrite( 8, HIGH );
      digitalWrite( 6, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, OUTPUT);  
      pinMode(7, INPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, INPUT);        
   }   
   else if( iLedNum == 9 )   
   {
      digitalWrite( 5, HIGH );
      digitalWrite( 8, LOW );
     
      pinMode(5, OUTPUT);  
      pinMode(6, INPUT);  
      pinMode(7, INPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, INPUT);        
   } 
   else if( iLedNum == 10 )   
   {
      digitalWrite( 8, HIGH );
      digitalWrite( 5, LOW );
     
      pinMode(5, OUTPUT);  
      pinMode(6, INPUT);  
      pinMode(7, INPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, INPUT);        
   }   
    else if( iLedNum == 11 )   
   {
      digitalWrite( 8, HIGH );
      digitalWrite( 7, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, INPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, INPUT);        
   } 
   else if( iLedNum == 12 )   
   {
      digitalWrite( 7, HIGH );
      digitalWrite( 8, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, INPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, INPUT);        
   }  
   else if( iLedNum == 13 )   
   {
      digitalWrite( 9, HIGH );
      digitalWrite( 7, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, INPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, INPUT);  
      pinMode(9, OUTPUT);        
   }   
   else if( iLedNum == 14 )   
   {
      digitalWrite( 7, HIGH );
      digitalWrite( 9, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, INPUT);  
      pinMode(7, OUTPUT);  
      pinMode(8, INPUT);  
      pinMode(9, OUTPUT);        
   } 
   else if( iLedNum == 15 )   
   {
      digitalWrite( 9, HIGH );
      digitalWrite( 8, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, INPUT);  
      pinMode(7, INPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, OUTPUT);        
   } 
   else if( iLedNum == 16 )   
   {
      digitalWrite( 8, HIGH );
      digitalWrite( 9, LOW );
     
      pinMode(5, INPUT);  
      pinMode(6, INPUT);  
      pinMode(7, INPUT);  
      pinMode(8, OUTPUT);  
      pinMode(9, OUTPUT);        
   }   
}


void BMA250Init()
{
  // Setup the range measurement setting
  Wire.beginTransmission(BMA250_I2CADDR);
  Wire.write(0x0F); 
  Wire.write(BMA250_RANGE);
  Wire.endTransmission();
  
  // Setup the bandwidth
  Wire.beginTransmission(BMA250_I2CADDR);
  Wire.write(0x10);
  Wire.write(BMA250_BW);
  Wire.endTransmission();
}

int BMA250ReadAccel()
{
  uint8_t ReadBuff[8];
  
  // Read the 7 data bytes from the BMA250
  Wire.beginTransmission(BMA250_I2CADDR);
  Wire.write(0x02);
  Wire.endTransmission();
  Wire.requestFrom(BMA250_I2CADDR,7);
  
  for(int i = 0; i < 7;i++)
  {
    ReadBuff[i] = Wire.read();
  }
  
  AccelX = ReadBuff[1] << 8;
  AccelX |= ReadBuff[0];
  AccelX >>= 6;
  
  AccelY = ReadBuff[3] << 8;
  AccelY |= ReadBuff[2];
  AccelY >>= 6;
  
  AccelZ = ReadBuff[5] << 8;
  AccelZ |= ReadBuff[4];
  AccelZ >>= 6;  

  AccelTemperature = (ReadBuff[6] * 0.5) + 24.0;
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
