/*  
 *   KRPAI 
*/
#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>;
//AMG set up
#include <Adafruit_AMG88xx.h>
Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];


#define pinFan 9 
//pinfan harus PWM
Servo propeller;  

#define CMPS11_ADDRESS 0x60  // Address of CMPS11 shifted right one bit for arduino wire library
#define ANGLE_8  1           // Register to read 8bit angle from

#define enA 3
#define in1 12
#define in2 13
#define enB 5
#define in3 4
#define in4 7
byte motorSpeedA = 0;
byte motorSpeedB = 0;

#define UVpin A4


#define button 11

//int Pingpin[8]=[A1,A2,A3,A4,A5,8,10,11];
//Pin ping (asumsi 6 ) 
#define pingFL A2 // Front Left
#define pingFM A1 // Front Middle 
#define pingFR A3 // Front Right 
#define pingL A0 // Left
#define pingR 10 // Right 
#define pingB 8  // Back 
NewPing FL(pingFL,pingFL,2000);
NewPing FM(pingFM,pingFM,2000);
NewPing FR(pingFR,pingFR,2000);
NewPing L(pingL,pingL,2000);
NewPing R(pingR,pingR,2000);
NewPing B(pingB,pingB,2000);

float datapingFL;
float datapingFM;
float datapingFR;
float datapingL;
float datapingR;
float datapingB;

// Random Var
int val;
int buttonState;

// Var getUV()
int sensorValue = 0;  // Variabel Penyimpan Input UVTron
int sensorYES = 400;  // Batas atas 
int sensorNO = 10;  // Batas bawah 
int adaApi = 0; // 0 = tidak ada api, 1 = ada api,(0,1) = ada bayangan
int numRead = 0; // 
int Max_numRead = 10; //

// Var getAMG()
bool status_amg;

// LIST FUNGSI 
/*float getCompass() ;
/*
 *  Output langsung nilai compass
 */
void motorMove(char motor,int dir,int speedmotor,int delayTime) ;
/*
 * motor = 'L' for left , 'R' for Right
 * dir = 1 for forward , 0 for backward, 3 for brake 
 * speedmotor range dari 0-100 (udah dimapping sebelumnya dari 0-255 PWM )
 * delayTime : Waktu delay (ms), setelah tiap action kalau mau berenti tetap harus di STOP dulu 
 * ex : 
 * motorMove('R',1,100,2500);
 * motorMove('R',3,0,50);
 */
void fanOut(int fanSpeed,bool fanStatus ,int delayTime);
/*
 * Buat nyalain kipas 
 * fanspeed dari 0-100 
 * fanStatus = 1 buat nyala , 0 buat mati 
 * delay = ms 
 * kalau mau matiin kipasnya tetep harus di stop 
 * ex : fanOut(0,0,50);
 */
void getAMG();
/*
 * Output nya langsung di array float pixels 
 */
 
void getUV();
/*
 * Output True or False , Algoritma fungsinya masih belom 100% jalan, nanti diganti .
 */

//void getPing();
/*
 * Update 6 sensor ultra ,disimpen di variabel
 */
 
//DECLARE FUNGSI
void getPing()
{
  datapingFL=FL.ping_cm();
  datapingFM=FM.ping_cm();
  datapingFR=FR.ping_cm();
  datapingL=L.ping_cm();
  datapingR=R.ping_cm();
  datapingB=B.ping_cm();
  Serial.println("START PING");
  Serial.print("Front left : ");
  Serial.println(datapingFL);
  Serial.print("Front mid : ");
  Serial.println(datapingFM);
  Serial.print("Front right : ");
  Serial.println(datapingFR);
    Serial.print("Left : ");
  Serial.println(datapingL);
    Serial.print("right : ");
  Serial.println(datapingR);
    Serial.print("Back : ");
  Serial.println(datapingB);
  
}

void getUV()
{
   // read the value from the sensor:
  sensorValue = analogRead(UVpin);
  if (sensorValue > sensorYES) {
      adaApi = 1;
      numRead = Max_numRead;
    }
  else if (sensorValue < sensorNO) {
      if (numRead > 0) {
        numRead = numRead - 1;
      } 
      else {
        adaApi = 0;
      }
    }
  //Serial.println(sensorValue);
  Serial.print("adaApi : ");
  Serial.println(adaApi);
  delay(10);
  
}

void getAMG()
{
  status_amg = amg.begin();
    if (!status_amg) {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");}
  amg.readPixels(pixels);

    Serial.print("[");
    for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
      Serial.print(pixels[i-1]);
      Serial.print(", ");
      if( i%8 == 0 ) Serial.println();
    }
    Serial.println("]");
    Serial.println();
}

float getCompass() 
{
unsigned char high_byte, low_byte, angle8;
char tmp; // Variabel untuk menyimpan data yang tidak diperlukan (Roll & Pitch)
unsigned int angle16; 
float heading;

 Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
  Wire.write(ANGLE_8);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 5 bytes from the CMPS11
  // this will give us the 8 bit bearing,
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS11_ADDRESS, 5);       
 
  while(Wire.available() < 5);        // Wait for all bytes to come back
 
  angle8 = Wire.read();               // Baca 3 Byte data kompas
  high_byte = Wire.read();
  low_byte = Wire.read();
  tmp = Wire.read();  // Abaikan 2 byte terahir (Data Roll & Pitch)
  tmp = Wire.read();
 
  angle16 = high_byte;                 // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += low_byte;
 
  heading = (angle16 / 10) + (angle16 % 10);
  //Serial.print("Sudut: ");
  //heading=map(heading,0,367,0,360);
 return heading;
 
}
void motorMove(char motor,int dir,int speedmotor) 
// motor = L (Left) or R (Right) 
// dir   = 1 for Maju , 0 for mundur 
// speedmotor = range input 0-100
{
  speedmotor=map(speedmotor,0,100,80,255);
  Serial.print("Motor ");
  if ( motor== 'L')
  {
    Serial.print("LEFT ");
    if (dir==1)
    {
      Serial.print("Forward with Speed ");
      Serial.print(speedmotor);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
            analogWrite(enA, speedmotor);
            
    }
    else if( dir==0)
    {
      Serial.print("Backward with Speed ");
      Serial.print(speedmotor);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, speedmotor);
    }
    else
    {
      Serial.print("STOP");
      digitalWrite(in1, HIGH);
      digitalWrite(in2, HIGH);
      analogWrite(enA, speedmotor);
    }
  }
  else if (motor == 'R')
  {
        Serial.print("RIGHT ");
        if (dir==1)
    {
            Serial.print("Forward with Speed ");
      Serial.print(speedmotor);
        analogWrite(enB, speedmotor);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }
    else if(dir==0)
    {
              Serial.print("Backward with Speed ");
      Serial.print(speedmotor);
        analogWrite(enB, speedmotor);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }
    else
    {
              Serial.print("STOP");
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
    }
  }
Serial.println();
}


void testrotate()
{
      int lastreading=getCompass();
    int currentreading;
    currentreading=getCompass();
    int condition= (currentreading+(360-lastreading))%360 ;
    while (not((condition >67) and (condition <73)))
    {
      currentreading=getCompass();
      condition= (currentreading+(360-lastreading))%360 ;
      Serial.print("the condition is still :");
      Serial.println(condition);
      Serial.print("Bacaan Sensor : ");
      Serial.println(currentreading);
      Serial.print("bacaan acuan : ");
      Serial.println(lastreading);
      delay(300);
    }
    Serial.println("Got out of Condition");
    delay(2000);
}
void fanOut(int fanSpeed,bool fanStatus ,int delayTime) //Fanspeed range 0-100 , fanStatus 1: nyala fanStatus 0:mati .
{

  if (fanStatus)
  {
    fanSpeed= map(fanSpeed,0,100,1000,2000);
    propeller.writeMicroseconds(fanSpeed);
  }
  else 
  {
    fanSpeed=0;
    propeller.writeMicroseconds(fanSpeed);
  }
  delay(delayTime);
}

// SETUP
void setup() {
  Wire.begin();
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(button,INPUT);
  pinMode(UVpin,INPUT);
  Serial.begin(9600);
  propeller.attach(pinFan,1000,2000); // attaches the servo on pin pinFan to the propeller object , minspeed = 1000, maxspeed = 2000 dalam microseconds
  // INISIASI AWAL PROPELLER 
  delay(1000);
  // Initialize Propeller ( Set max freq )
  propeller.writeMicroseconds(2000);
    delay(1000);
    propeller.writeMicroseconds(1000);
    delay(  1000);
  // Kalo motor nya uda bunyi 3x berarti udah di set 
  propeller.writeMicroseconds(0);

//Button activation
int buttonState=0;
Serial.println("Robot Start");
/*while (buttonState==0)
{
  buttonState=digitalRead(button);
 // Serial.println("button : OFF ");
}*/
Serial.println("Robot activated");

//Start AMG
//amg.begin();

}


void loop() 
{
  Serial.println("initiate");
getPing();
delay(1000);
  motorMove('R',3,80);
  motorMove('L',3,80);
delay(1000);
motorMove('R',0,80);
  motorMove('L',0,80);
delay(1000);

  motorMove('R',3,80);
  motorMove('L',3,80);
delay(1000);
}