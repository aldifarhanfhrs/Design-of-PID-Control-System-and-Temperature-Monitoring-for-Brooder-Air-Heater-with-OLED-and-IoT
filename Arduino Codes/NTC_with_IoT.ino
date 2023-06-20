 //kalibrasi ESP32 ada di https://forum.arduino.cc/t/ide-and-ntc-thermistor/683955/17

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <EEPROM.h>
#include <MQ135.h>
#include <RBDdimmer.h>
#include "UbidotsEsp32Mqtt.h"
#include <PubSubClient.h>
//#include <TimerOne.h>

//--------------------------------------------------------------------------------
#define btnUp   33
#define btnOk   35
#define btnDown 32
#define btnBack 34
#define ThermistorPin 25
#define MQ135Pin 26
#define outputPin  19 
#define zerocross  18

#define RZERO 206.85

#define ledRed 14
#define ledGreen 12
#define buzzer 27


int Vo;
int outVal = 0;
float nilaiMQ;
float R1 = 10000;
float logR2, R2, T, Tc, Tf;
float c1 = 1.145226154e-3, c2 = 2.296821079e-4, c3 = 1.194648779e-7;

MQ135 gasSensor = MQ135(MQ135Pin);
Adafruit_SSD1306 lcd(128, 64, &Wire, -1);
dimmerLamp dimmer(outputPin, zerocross);
//--------------------------------------------------------------------------------
int Kp, Ki, Kd;
int SV;

int SV_Value  = 0;
int Kp_Value  = 1;
int Ki_Value  = 2;
int Kd_Value = 3;
//double Error;

float Error, lastError, sumError, PID;
//--------------------------------------------------------------------------------
bool statusBtnUp   = false;
bool statusBtnOk   = false;   
bool statusBtnDown = false;
bool statusBtnBack = false;

bool statusAkhirBtnUp   = false;
bool statusAkhirBtnOk   = false;
bool statusAkhirBtnDown = false;
bool statusAkhirBtnBack = false;

bool UP   = false;
bool _OK   = false;
bool DOWN = false;
bool BACK = false;

volatile bool buttonPressedUp = false;
volatile bool buttonPressedDown = false;
volatile bool buttonPressedOK = false;
volatile bool buttonPressedBack = false;

//--------------------------------------------------------------------------------
int halaman  = 1;
int menuItem = 1;
//--------------------------------------------------------------------------------


void IRAM_ATTR handleInterruptUp() {
  buttonPressedUp = true;
}

void IRAM_ATTR handleInterruptDown() {
  buttonPressedDown = true;
}

void IRAM_ATTR handleInterruptOK() {
  buttonPressedOK = true;
}

void IRAM_ATTR handleInterruptBack() {
  buttonPressedBack = true;
}
//--------------------------------------------------------------------------------
float ntcRead(){
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (4095.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  Tc = T - 273.15;

  return Tc;
  delay(100);
}

//----------------------------------------------------------------------------
void mqRead(){
  float rzero = gasSensor.getRZero();
  nilaiMQ = gasSensor.getPPM();
    if (nilaiMQ > 5000) {
    digitalWrite(buzzer, HIGH);
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledGreen, LOW);

  } else {
    digitalWrite(ledRed, LOW);
    digitalWrite(ledGreen, HIGH);
    digitalWrite(buzzer, LOW);
  }
}
//--------------------------------------------------------------------------------
void pidDimmer (){
  lastError = Error;
  Error = SV - ntcRead();
  sumError += Error;

  PID = (Kp * Error) + ( Ki * sumError) + ((Kd/100)*(Error - lastError));
  //pvPower = svPower - PID;

  if (PID > 100){
    PID = 100;
  }
  else if (PID < 0){
    PID = 0;
  }
  
  dimmer.setPower(PID);
  //delay(100);
}
//--------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  lcd.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  dimmer.begin(NORMAL_MODE, ON);

  lcd.clearDisplay();

  pinMode(btnUp,   INPUT_PULLUP);
  pinMode(btnOk,   INPUT_PULLUP);
  pinMode(btnDown, INPUT_PULLUP);
  pinMode(btnBack, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(btnUp), handleInterruptUp, FALLING);   // Menyambungkan interupsi dengan pin push button 1
  attachInterrupt(digitalPinToInterrupt(btnOk), handleInterruptOK, FALLING);   // Menyambungkan interupsi dengan pin push button 2
  attachInterrupt(digitalPinToInterrupt(btnDown), handleInterruptDown, FALLING);   // Menyambungkan interupsi dengan pin push button 3
  attachInterrupt(digitalPinToInterrupt(btnBack), handleInterruptBack, FALLING);

  pinMode (ledRed, OUTPUT);
  pinMode (ledGreen, OUTPUT);
  pinMode (buzzer, OUTPUT);

  pinMode (MQ135Pin, INPUT);
  
  //deklarasi EEPROM Kp, Ki, Kd
  EEPROM.begin(512);
  Kp = EEPROM.read(Kp_Value);
  Ki = EEPROM.read(Ki_Value);
  Kd = EEPROM.read(Kd_Value);
  SV = EEPROM.read(SV_Value);

  if (Kp >= 255) {Kp=0;}
  if (Ki >= 255) {Ki=0;}
  if (Kd >= 255) {Kd=0;}
  if (SV >= 255) {SV=0;}


}
//--------------------------------------------------------------------------------
void loop() {
  //ntcRead();
  mqRead();
  tampil();
  pidDimmer();

  statusBtnUp   = digitalRead(btnUp);
  statusBtnOk   = digitalRead(btnOk);
  statusBtnDown = digitalRead(btnDown);
  statusBtnBack = digitalRead(btnBack);

  if (buttonPressedUp) {
    buttonPressedUp = false;
    handleButtonUp();
  }

  if (buttonPressedDown) {
    buttonPressedDown = false;
    handleButtonDown();
  }

  if (buttonPressedOK) {
    buttonPressedOK = false;
    handleButtonOK();
  }

  if (buttonPressedBack) {
    buttonPressedBack = false;
    handleButtonBack();
  }
  //delay(50);

  saatUpDitekan();
  saatOkDitekan();
  saatDownDitekan();
  saatBackDitekan();
  // Delay singkat untuk stabilisasi tombol
  
  Serial.print (lastError);
  Serial.print("\t");
  Serial.print (Error);
  Serial.print("\t");
  Serial.print (sumError);
  Serial.print("\t");
  Serial.print (SV);
  Serial.print("\t");
  Serial.print (ntcRead());
  Serial.print("\t");
  Serial.print(PID);
  Serial.print("\t");
  Serial.println (nilaiMQ);
  //delay(100);


}
//--------------------------------------------------------------------------------
void handleButtonUp() {
    if (UP && halaman == 1) {
    UP = false;
    menuItem --;
    if (menuItem < 1)menuItem = 5;
  }
}
void handleButtonDown() {
    if (DOWN && halaman == 1) {
    DOWN = false;
    menuItem ++;
    if (menuItem > 5)menuItem = 1;
  }
}
void handleButtonOK() {
    if (_OK == true) {
    _OK = false;
    if (halaman == 1 && menuItem == 1) {
      halaman = 2;
    } else if (halaman == 1 && menuItem == 2) {
      halaman = 3;
    } else if (halaman == 1 && menuItem == 3) {
      halaman = 4;
    } else if (halaman == 1 && menuItem == 4) {
      halaman = 5;
    } else if (halaman == 1 && menuItem == 5) {
      halaman = 6;
    }
  }
}
void handleButtonBack() {
    if (BACK == true) {
    BACK = false;
    if (halaman == 2 || halaman == 3 || halaman == 4 || halaman == 5 || halaman == 6) {
    halaman = 1;
    }
  }
}
//----------------------------------------------------------------------------------
void saatUpDitekan() {
  if (statusBtnUp != statusAkhirBtnUp) {
    if (statusBtnUp == 0) {
      UP = true;
    }
    delay(50);
  }
  statusAkhirBtnUp = statusBtnUp;
}

void saatOkDitekan() {
  if (statusBtnOk != statusAkhirBtnOk) {
    if (statusBtnOk == 0) {
      _OK = true;
    }
    delay(50);
  }
  statusAkhirBtnOk = statusBtnOk;
}

void saatDownDitekan() {
  if (statusBtnDown != statusAkhirBtnDown) {
    if (statusBtnDown == 0) {
      DOWN = true;
    }
    delay(50);
  }
  statusAkhirBtnDown = statusBtnDown;
}

void saatBackDitekan() {
  if (statusBtnBack != statusAkhirBtnBack) {
    if (statusBtnBack == 0) {
      BACK = true;
    }
    delay(50);
  }
  statusAkhirBtnBack = statusBtnBack;
}
//----------------------------------------------------------------------------

//semua yang tampil di lcd ada di fungsi ini
void tampil() {
  if (halaman == 1) {
    lcd.clearDisplay();
    lcd.setTextSize(1);
    lcd.setTextColor(WHITE);
    lcd.setCursor(17, 0); //(X, Y);
    lcd.print("PID CONTROL MENU");
    lcd.setCursor(0, 7); //(X, Y);
    lcd.print("_____________________");

    if (menuItem == 1) {
      lcd.setCursor(5, 17);
      lcd.setTextColor(WHITE);
      lcd.print("=> Monitoring    (C)");
    } else {
      lcd.setCursor(5, 17);
      lcd.setTextColor(WHITE);
      lcd.print("   Monitoring    (C)");
    }

    if (menuItem == 2) {
      lcd.setCursor(5, 27);
      lcd.setTextColor(WHITE);
      lcd.print("=> Set Value    (SV)");
    } else {
      lcd.setCursor(5, 27);
      lcd.setTextColor(WHITE);
      lcd.print("   Set Value    (SV)");
    }

    if (menuItem == 3) {
      lcd.setCursor(5, 37);
      lcd.setTextColor(WHITE);
      lcd.print("=> Proporsional (Kp)");
    } else {
      lcd.setCursor(5, 37);
      lcd.setTextColor(WHITE);
      lcd.print("   Proporsional (Kp)");
    }

    if (menuItem == 4) {
      lcd.setCursor(5, 47);
      lcd.setTextColor(WHITE);
      lcd.print("=> Integral     (Ki)");
    } else {
      lcd.setCursor(5, 47);
      lcd.setTextColor(WHITE);
      lcd.print("   Integral     (Ki)");
    }

    if (menuItem == 5) {
      lcd.setCursor(5, 57);
      lcd.setTextColor(WHITE);
      lcd.print("=> Diferential  (Kd)");
    } else {
      lcd.setCursor(5, 57);
      lcd.setTextColor(WHITE);
      lcd.print("   Diferential  (Kd)");
    }
  //################################################
  } else if (halaman == 2) {
    lcd.clearDisplay();
    lcd.setTextColor(WHITE);
    lcd.setTextSize(1);
    lcd.setCursor(0, 0);
    lcd.print("PV:");
    lcd.setTextSize(2);
    lcd.setCursor(0, 10);
    lcd.print(Tc,1);
    
    lcd.setTextSize(1);
    lcd.setCursor(0, 32);    
    lcd.print("Err:");
    lcd.setTextSize(2);
    lcd.setCursor(0, 42);
    lcd.print(Error,1); //(Tc,1)
    delay(100);
        
    lcd.setTextSize(1);
    lcd.setCursor(65, 0);
    lcd.print("SV:");
    lcd.setTextSize(2);
    lcd.setCursor(84, 0);
    lcd.print(SV,1);

    lcd.setTextSize(1);
    lcd.setCursor(64, 17);
    lcd.print("Kp :");
    lcd.print(Kp);

    lcd.setTextSize(1);
    lcd.setCursor(64, 27);
    lcd.print("Ki :");
    lcd.print(Ki);

    lcd.setTextSize(1);
    lcd.setCursor(64, 37);
    lcd.print("Kd :");
    lcd.print(Kd);

    lcd.setTextSize(1);
    lcd.setCursor(64, 47);
    lcd.print("PID:");
    lcd.print(PID,1);

    lcd.setTextSize(1);
    lcd.setCursor(64, 57);
    lcd.print("PPM:");
    lcd.print(nilaiMQ,1);
    
  //################################################    
  } else if (halaman == 3) {
    SV = EEPROM.read(SV_Value);

      lcd.clearDisplay();
      lcd.setTextSize(2);
      lcd.setTextColor(WHITE);
      lcd.setCursor(0, 0);
      lcd.println("Set Value");
      lcd.setCursor(0, 20);
      lcd.setTextSize(4);
      lcd.print(SV);

      if (digitalRead(btnUp) == LOW){
        delay(20);
        SV --;
        //if(SV > 255) SV = 1;
        EEPROM.write(SV_Value, SV);
        EEPROM.commit();
        lcd.display();
      }
      if (digitalRead(btnDown) == LOW){
        delay(20);
        SV ++;
        //if(SV < 0) SV = 255;
        EEPROM.write(SV_Value, SV);
        EEPROM.commit();
        lcd.display();
      }
      //}
    
  //################################################    
  } else if (halaman == 4) {
    Kp = EEPROM.read(Kp_Value);

      lcd.clearDisplay();
      lcd.setTextSize(2);
      lcd.setTextColor(WHITE);
      lcd.setCursor(0, 0);
      lcd.println("Kp Value");
      lcd.setCursor(0, 20);
      lcd.setTextSize(4);
      lcd.print(Kp);

      if (digitalRead(btnUp) == LOW){
        delay(20);
        Kp--;
        //if(SV > 255) SV = 1;
        EEPROM.write(Kp_Value, Kp);
        EEPROM.commit();
        lcd.display();
      }
      if (digitalRead(btnDown) == LOW){
        delay(20);
        Kp++;
        //if(SV < 0) SV = 255;
        EEPROM.write(Kp_Value, Kp);
        EEPROM.commit();
        lcd.display();
      }
  //################################################
  } else if (halaman == 5) {
      Ki = EEPROM.read(Ki_Value);

      lcd.clearDisplay();
      lcd.setTextSize(2);
      lcd.setTextColor(WHITE);
      lcd.setCursor(0, 0);
      lcd.println("Ki Value");
      lcd.setCursor(0, 20);
      lcd.setTextSize(4);
      lcd.print(Ki);

      if (digitalRead(btnUp) == LOW){
        delay(20);
        Ki --;
        //if(SV > 255) SV = 1;
        EEPROM.write(Ki_Value, Ki);
        EEPROM.commit();
        lcd.display();
      }
      if (digitalRead(btnDown) == LOW){
        delay(20);
        Ki ++;
        //if(SV < 0) SV = 255;
        EEPROM.write(Ki_Value, Ki);
        EEPROM.commit();
        lcd.display();
      }
  //################################################
  } else if (halaman == 6) {
      Kd = EEPROM.read(Kd_Value);

      lcd.clearDisplay();
      lcd.setTextSize(2);
      lcd.setTextColor(WHITE);
      lcd.setCursor(0, 0);
      lcd.println("Kd Value");
      lcd.setCursor(0, 20);
      lcd.setTextSize(4);
      lcd.print(Kd);

      if (digitalRead(btnUp) == LOW){
        delay(20);
        Kd --;
        //if(SV > 255) SV = 1;
        EEPROM.write(Kd_Value, Kd);
        EEPROM.commit();
        lcd.display();
      }
      if (digitalRead(btnDown) == LOW){
        delay(20);
        Kd ++;
        //if(SV < 0) SV = 255;
        EEPROM.write(Kd_Value, Kd);
        EEPROM.commit();
        lcd.display();
      }
  }

  lcd.display();
}

//---------------------------------------------------------------------------------