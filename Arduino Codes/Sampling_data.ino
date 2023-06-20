#include <RBDdimmer.h>

#define ThermistorPin 25
#define outputPin  19 
#define zerocross  18
int Vo;
int outVal = 0;

float R1 = 10000;
float logR2, R2, T, Tc, Tf;
float c1 = 1.145226154e-3, c2 = 2.296821079e-4, c3 = 1.194648779e-7;

dimmerLamp dimmer(outputPin, zerocross);
//-------------------------------------------------------------------
void setup(){
  //pinMode(ThermistorPin, 4);
  Serial.begin(9600);
  dimmer.begin(NORMAL_MODE, ON);
}
//-------------------------------------------------------------------
void printSpace(int val)
{
  if ((val / 100) == 0) Serial.print(" ");
  if ((val / 10) == 0) Serial.print(" ");
}
//-------------------------------------------------------------------
void loop(){
  ntcRead();

  int preVal = outVal;

  if (Serial.available())
  {
    int buf = Serial.parseInt();
    if (buf != 0) outVal = buf;
    delay(200);
  }
  dimmer.setPower(outVal);
  delay(100);
}
//-------------------------------------------------------------------
void ntcRead(){
  Vo = analogRead(ThermistorPin);
  //Serial.print ("  Vo =");
  //Serial.print(Vo);
  R2 = R1 * (4095.0 / (float)Vo - 1.0);
  //Serial.print("\t R2 =");
  //Serial.print(R2);
  logR2 = log(R2);
  //Serial.print("\t  logR2 =");
  //Serial.print(logR2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  //Serial.print("\t  T =");
  //Serial.print(T);
  Tc = T - 273.15;
  // Tf = (Tc * 9.0)/ 5.0 + 32.0;


  //Serial.print("\t  Temperature: ");
  Serial.println(Tc);
  //Serial.println(" C");

  delay(100);
}
//------------------------------------------------------------------------

