#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <LiquidCrystal.h> 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>  
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
void  Encoder_san();
//==============================================
//Set Encoder pin
//==============================================
const int Encoder_A =  3;            // Incremental Encoder singal A is PD3 
const int Encoder_B =  2;            // Incremental Encoder singal B is PD2 
//const int ledPin    =  13;  
unsigned int Encoder_number=0;
 int state=0;
 
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);   

SoftwareSerial mySerial(11, 12); // RX, TX
TinyGPS gps;

void gpsdump(TinyGPS &gps);
float printFloat(double f, int digits = 6);
float mph;
float alt;
long lat, lon;
float flat, flon;
unsigned long age, date, time, chars;
byte month, day, hour, minute, second, hundredths;
int level;
int year;
void setup()  
{
  
 // Open serial communications and wait for port to open:
 
  Serial.begin(9600);
  lcd.begin(20, 4);   // start the library  
  pinMode(10,OUTPUT);
  digitalWrite(10, 1);

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  
  
    pinMode(Encoder_A, INPUT); 
  pinMode(Encoder_B, INPUT); 
  digitalWrite(Encoder_A, 1);
  digitalWrite(Encoder_B, 1);
  //========================================
  attachInterrupt(1, Encoder_san, FALLING);        //interrupts: numbers 0 (on digital pin 2) and 1 (on digital pin 3).
 
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
}

void loop() // run over and over
{
  sensors_event_t event; 
  mag.getEvent(&event);
   if (Encoder_number > 5) {
      Encoder_number = 0;
    }
  
  
    lcd.clear();
   switch (Encoder_number) {
    case 1:
       lcd.setCursor(0,0);              
       lcd.print("Speed (mph)");  
       lcd.setCursor(0,1); 
       lcd.print(mph);
      break;
    case 2:
       lcd.setCursor(0,0); 
       lcd.print("Date: "); lcd.print(static_cast<int>(month)); lcd.print("/"); 
       lcd.print(static_cast<int>(day)); lcd.print("/"); lcd.print(year); 
       lcd.setCursor(0,1); 
       lcd.print("Time: "); lcd.print(static_cast<int>(hour)); lcd.print(":"); 
       lcd.print(static_cast<int>(minute)); lcd.print(":"); lcd.print(static_cast<int>(second));
       lcd.print("."); lcd.print(static_cast<int>(hundredths));
      break;
     case 3:
       lcd.setCursor(0,0);            
       lcd.print("Altitude"); 
       lcd.setCursor(0,1); 
       lcd.print(alt);
      break;
      case 4:
       lcd.setCursor(0,0);            
       lcd.print("Level"); 
       lcd.setCursor(0,1); 
       lcd.print(level);
      break;
      case 5:
        lcd.setCursor(0,0);
        lcd.print("X: ");
        lcd.print(event.magnetic.x); 
        lcd.setCursor(0,1);
        lcd.print("Y: ");
        lcd.print(event.magnetic.y); 
        lcd.setCursor(0,2);
        lcd.print("Z: ");
        lcd.print(event.magnetic.z);
        break;
    default: 
       lcd.setCursor(0,0);            
       lcd.print("Long ");
       lcd.print(flon, 6);
       lcd.setCursor(0,1);            
       lcd.print("Lat ");
       lcd.print(flat, 6);
      
        
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.22;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  float headingDegrees = heading * 180/M_PI; 
  lcd.setCursor(0,2);
  lcd.print("Head (deg): "); lcd.print(headingDegrees);
  
 
  }
  
  
  

    
    
  bool newdata = false;
  unsigned long start = millis();

  // Every 5 seconds we print an update
  while (millis() - start < 5000) {
    if (mySerial.available()) {
      char c = mySerial.read();
      // Serial.print(c);  // uncomment to see raw GPS data
      if (gps.encode(c)) {
        newdata = true;
         break;  // uncomment to print new data immediately!
      }
    }
  }
  
  if (newdata) {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    Serial.println("-------------");
    Serial.println();
  }
  
}

void gpsdump(TinyGPS &gps)
{
  //long lat, lon;
  //float flat, flon;
  //unsigned long age, date, time, chars;
  //int year;
  //byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  // On Arduino, GPS characters may be lost during lengthy Serial.print()
  // On Teensy, Serial prints to USB, which has large output buffering and
  //   runs very fast, so it's not necessary to worry about missing 4800
  //   baud GPS characters.

  gps.f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); printFloat(flat, 7); Serial.print(", "); printFloat(flon, 7);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.get_datetime(&date, &time, &age);
  Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): ");
    Serial.print(time);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); 
    Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour)); Serial.print(":"); 
    Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));
    Serial.print("."); Serial.print(static_cast<int>(hundredths));
  Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");

  Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): ");
    Serial.print(gps.course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(gps.speed());
  Serial.print("Alt(float): "); printFloat(gps.f_altitude()); Serial.print(" Course(float): ");
    printFloat(gps.f_course()); Serial.println();
  Serial.print("Speed(knots): "); printFloat(gps.f_speed_knots()); Serial.print(" (mph): ");
    printFloat(gps.f_speed_mph());
  Serial.print(" (mps): "); printFloat(gps.f_speed_mps()); Serial.print(" (kmph): ");
    printFloat(gps.f_speed_kmph()); Serial.println();
    
    mph=printFloat(gps.f_speed_mph());
    alt=printFloat(gps.f_altitude());
    

  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: ");
    Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);



}

float printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.00000) {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  int rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}

 
 void Encoder_san()
{  
 
        if(digitalRead(Encoder_B))
          {
             Encoder_number++;
          }
        else
          {  
            Encoder_number--;
          }     
          state=1;
}
