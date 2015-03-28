/*

    28.03.2015
    
    This is a fork of Alain De Carolis, WW3WW, FT-817 Automatic Loop Tuner (https://code.google.com/p/ft-817-automatic-loop/)
   
    This progam uses a modified version of the qrptracker library
    written by Bruce Gordon Robertson, VE9QRP
    
    released under the GPL V3 license
    
    I have added a 16x2 LCD display with some infos of the rig an the possibility to work with a digital interface and 
    this tuner.
    
    
    
*/

#include <SoftwareSerial.h>
#include "FT817.h"
#include <PWMServo.h> 
#include <EEPROM.h>

//i2c lcd
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define I2C_ADDR    0x27 
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

LiquidCrystal_I2C	lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
//i2c lcd

const int addr = 0;            // EEPROM address for saving the current capacitor position

int n = LOW;
int encoder0PinALast = LOW;

const int encoder0PinA = 3;    // Encoder PinA
const int encoder0PinB = 4;    // Encoder PinB
const int stepTime = 100;      // Mesures slow or fast encoder rotation
const int bigStep = 5;        // degrees during fast rotation --> orig was 10!!
const int smallStep = 1;       // degrees during slow rotation
int step;

const int rxPin = 8;
const int txPin = 7;
const int relayPin = 6;        // Pin for relay to switch DataIN/DataOUT

const int buttonPin = 5;       // tuning button - push to tune
int buttonState = 0;           // tune button

const int ledPin = 12;         // tuning led - ON when tuning was successul
const int ledPinON = 11;       // tuned led - ON when tuned OK - blink for errors
const int servoPin = 9;        // this MUST be 9 when using the PWMServo lib
const int buzzerPin = 10;      // some acoustic effects
const int servoTimeout = 2000; // 2 seconds unused and the servo will detach

unsigned long servoTime;       // used to decide when we can detach the servo
unsigned long encoderTime;     // used to determine the econcoder rotation speed

int pos;                       // Servo Position helper variable
int swr;                       // SWR helper variable
unsigned long frq;             // Frequenz from rig
const int minPos = 1;          // Minimum Position reachable by the Servo
const int maxPos = 180;        // Maximum Position reachable by the Servo

PWMServo myservo;
FT817 rig;

void setup() {
  
    lcd.begin (16,2);
    lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
    lcd.setBacklight(HIGH);
    lcd.home (); // go home
    lcd.print("Arduino ML Tuner");
    lcd.setCursor(1, 1);
    lcd.print("----OE8KUR----");
    delay(1500);
    init_screen();

    pinMode (encoder0PinA,INPUT);
    pinMode (encoder0PinB,INPUT);
    pinMode(buttonPin, INPUT);
    pinMode(buttonPin, INPUT);
    
    pinMode(ledPin, OUTPUT);
    pinMode(ledPinON, OUTPUT);


    digitalWrite(relayPin, HIGH); 
    //digitalWrite(encoder0PinA, HIGH);
    //digitalWrite(encoder0PinB, HIGH);
  
    Serial.begin(9600);
    SoftwareSerial mySerial(rxPin,txPin);
    rig.assignSerial(mySerial);
    

    
    digitalWrite(ledPinON, HIGH);
    
    pos = EEPROM.read(addr);
    myservo.attach(servoPin);
    myservo.write(pos);
    Serial.print("Servo positioned to ");
    Serial.println(pos);
    lcd_update_pos();
    servoTime = millis();
  
}

void loop() {
  
    rig.begin(9600);
    
    frq = rig.getFreqMode();
    lcd_update_freq();

    
    if(( myservo.attached() ) && (millis() > (servoTime + servoTimeout))) {
      myservo.detach();
      Serial.println("Detaching the Servo");
      EEPROM.write(addr, pos);
      Serial.println(EEPROM.read(addr));
      lcd_update_pos();
    }
  
    buttonState = digitalRead(buttonPin);

    if (buttonState == HIGH) {
        tune();
    }
      
    n = digitalRead(encoder0PinA);
    
    if ((encoder0PinALast == LOW) && (n == HIGH)) {
      
      if (! myservo.attached() ) {
        myservo.attach(servoPin);
        myservo.write(pos);
      }
           
      if (millis() - encoderTime > stepTime) {
        step = smallStep;
      } else {
        step = bigStep;
      }
      
      encoderTime = millis();
      
      if (digitalRead(encoder0PinB) == LOW) {
        
        if( pos >= maxPos ) { pos = maxPos; singleFlash(); }
        else {
          pos += step;
        }
        
      } else { // digitalRead(encoder0PinB) == HIGH
        
        if( pos <= minPos ) { pos = minPos; singleFlash(); }
        else {
          pos -= step;
        }
        
      }
     
     Serial.print("Position selected by the Encoder: ");
     Serial.println (pos);
     lcd_update_pos();
     myservo.write(pos);
     servoTime = millis();
     
   }
   encoder0PinALast = n;
     
}

void tune() {

    boolean tuned = 0;
    unsigned int quadrant = 0;
    unsigned long sweepTime;
    unsigned int quadrantSize = 9;  // 15  - 9
    unsigned quadrantTime = 150;  // 150  - 200
  
    Serial.println("At Tune...");

    byte mode = rig.getMode();
    Serial.print("The Radio is in mode ");
    Serial.println(mode);

    rig.setMode(FT817_MODE_FM);
    
    rig.setPTTOn();
    delay(300);
     
    swr = rig.getSWR();
    
    Serial.print("Initial SWR ");
    Serial.println(swr);
    lcd_update_swr();
    
    if (! rig.getTXSuccess()) {
      rig.setPTTOff();
      delay(300);
      rig.setMode(mode);
      Serial.println("Error: The Radio cannot transmit!");
      errorBeep();
      return;
    }
    
    if (swr <= 2) {   //was 3
      rig.setPTTOff();
      delay(300);
      rig.setMode(mode);
      Serial.println("Already Tuned");
      successBeep();
      return;
    }
    
    Serial.println("Moving the Capacitor");
    if (! myservo.attached() ) {
      myservo.attach(servoPin);
      servoTime = millis();
    }

    // Move to one end
    // TODO: choose the right end depending on where the capacitor
    // already is
    
    myservo.write(minPos);
    servoTime = millis();
    
    // give some time to the servo to reach its position
    // a standard HS-322HD does 0.19 sec/60° at 4.8V
    
    delay(1500); 
    
    Serial.println("Starting the SWR Samples");
    // This is faster than sampling one degree at a time
    for( pos = minPos; pos <= maxPos; pos += quadrantSize ) {
      
      quadrant++;

      Serial.print("Entering quadrant ");
      Serial.print(quadrant);
      Serial.print(": ");
      Serial.print(quadrantSize * (quadrant -  1) + minPos);
      Serial.print("-");
      Serial.println(quadrantSize * quadrant + minPos);
      
      myservo.write(pos);
      
      servoTime = millis();

      for(sweepTime = servoTime; sweepTime <= servoTime + quadrantTime; sweepTime = millis()) {
        
        // Stop if a button is pressed
        if ( digitalRead(buttonPin) == HIGH ) {
          errorBeep();
          break;
        }
          
        swr = rig.getSWR();
        Serial.print(" SWR: ");
        Serial.println(swr);
        lcd_update_swr();
  
        if (swr <= 0) {   // alter wert war == 0
              
          Serial.println(" SWR 0 Found!");
          tuned = 1;
          break;
              
        }

      }
      
      if (tuned) {
        Serial.print("Tunable in quadrant ");
        Serial.println(quadrant);
        
        // Fine Tuning
        // Go back two quadrants to find the right position
        tuned = 0;
        for (pos = quadrantSize * quadrant + minPos; 
             pos >= quadrantSize * ( quadrant - 2) + minPos;
             pos--) {
          
          myservo.write(pos);
          Serial.print("Trying pos ");
          Serial.print(pos);
          lcd_update_pos();
          delay(50);
          swr = rig.getSWR();
          Serial.print(" SWR: ");
          Serial.println(swr);
          lcd_update_swr();
          
          if (swr <= 1) {
             Serial.print("Tuned to pos ");
             Serial.println(pos);
             lcd_update_pos();
             lcd_update_swr();
             tuned = 1;
             break;
          } 
          
        }
        
        break;   

      } 
      
    }
        
    // The End
    // back to RX
    rig.setPTTOff();
    delay(300);
    // Switch back to original mode
    rig.setMode(mode);
    Serial.print("Mode tuned: ");
    Serial.println(mode);
    
    if (tuned) {
      successBeep();
    } else {
      errorBeep();
    }
    
}

void singleFlash() {
   
   digitalWrite(ledPinON, LOW);
   delay(300);
   digitalWrite(ledPinON, HIGH);   
   
}

void errorBeep() {
   
   digitalWrite(ledPinON, LOW);
   delay(300);
   digitalWrite(ledPinON, HIGH);
   delay(300);
   digitalWrite(ledPinON, LOW);
   delay(300);
   digitalWrite(ledPinON, HIGH);
   delay(300);   
   tone(buzzerPin, 800, 500);
   
}

void successBeep() {
  
   digitalWrite(ledPin, HIGH);

   tone(buzzerPin, 600, 500);
   digitalWrite(ledPin, HIGH);
   delay(100);
   digitalWrite(ledPin, LOW);
   
   tone(buzzerPin, 800, 500);
   digitalWrite(ledPin, HIGH);
   delay(100);
   digitalWrite(ledPin, LOW);

   tone(buzzerPin, 1000, 500);
   digitalWrite(ledPin, HIGH);
   delay(100);
   
   digitalWrite(ledPin, LOW);
   
}

void init_screen()
{
  lcd.clear(); 
  lcd.setCursor (0,0); 
  lcd.print("SWR: ");
  lcd.setCursor (5,0); 
  lcd.print("   ");
  
  lcd.setCursor(9,0);
  lcd.print("Pos: ");  
  lcd.print("   ");
  
  lcd.setCursor(0,1);
  lcd.print("FRQ: ");  
  lcd.print("    ");
 }
 
 void lcd_update_pos()
 {
  lcd.setCursor (13,0);
  if (pos < 100){
    lcd.print(" ");
    }
    if (pos < 10){
    lcd.print(" ");
    }
    lcd.print (pos);
 }
 
 void lcd_update_freq()
 {
  lcd.setCursor (4,1);
  //t_frq = frq/100;
  if (frq < 1000000){
    lcd.print(" ");
    }
  if (frq < 100000){
    lcd.print(" ");
    } 
  if (frq < 10000){
    lcd.print(" ");
    } 

   lcd.print((frq/100), DEC);
   lcd.print(",");
   lcd.print((frq)%100, DEC); 
   lcd.print(" kHz");  
   if((frq)%100 < 10)
   {
     lcd.setCursor (15,1);
     lcd.print(" ");
   }
  }
 
 void lcd_update_swr()
 {
  lcd.setCursor(4,0);
  lcd.print(swr);
  } 
 
 
