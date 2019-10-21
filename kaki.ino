#include <Wire.h>;
#define CMPS11_ADDRESS 0x60  // Address of CMPS11 shifted right one bit for arduino wire library
#define ANGLE_8  1           // Register to read 8bit angle from
#include <Adafruit_AMG88xx.h>
Adafruit_AMG88xx amg;
//PIN SENSOR
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
const byte signalPing[12] = {24, 22, A15, A9, A7, A3, 12, 11, 10, 7, 2, 23};
const byte signalIR[2] = {A14, A13};
const byte signalUV[2] = {A12, A11};
const byte signalMic = 16;
const byte signalLine = 34;
// VARIABLE
int received1,received2,received3,received4,received5;
byte x;
int val = 0;
bool sound = 1;
unsigned long pulseduration = 0;
unsigned char high_byte, low_byte, angle8;
char tmp; // Variabel untuk menyimpan data yang tidak diperlukan (Roll & Pitch)
unsigned int angle16;
float heading;
unsigned long time1,time2,delta,timer1,timer2,delta2;
//Penyimpanan Data
float DataSent[81];
float Ping[12];
float IR[2];
float AMG[64];
int line,UVtron,mic;
float compass;

// PING
float measureDistance(int signalPing)
{
  // menetapkan pin sebagai output sehingga kita dapat mengirimkan pulsa
  pinMode(signalPing, OUTPUT);

  // set output menjadi LOW
  digitalWrite(signalPing, LOW);
  delayMicroseconds(1);

  // sekarang mengirim 5us pulsa untuk mengaktifkan Ping
  digitalWrite(signalPing, HIGH);
  delayMicroseconds(1);
  digitalWrite(signalPing, LOW);

  // sekarang kita perlu mengubah pin digital input untuk membaca pulsa masuk
  pinMode(signalPing, INPUT);

  // mengukur panjang pulsa masuk
  pulseduration = pulseIn(signalPing, HIGH);
  Serial.print("ping :");
  Serial.println(pulseduration);
  return pulseduration;

}
void getPing()
{
  Serial.println("PING");
  for (int i = 0; i < 12; i++) {
    Serial.print(i);
    Ping[i]=measureDistance(signalPing[i]);
  }
}
void getIR()
{
  Serial.println("IR");
  for (int i = 0; i < 2; i++) {
    val = analogRead(signalIR[i]);
    IR[i]=val;
    Serial.println(val);
  }
}
void getUV()
{
  Serial.println("UV");
  for (int i = 0; i < 2; i++) {
    val = digitalRead(signalUV[i]); 
    UVtron=val;
    if ( val == 1) {
      Serial.println("Ada api");
    }
    else(val == 0); {
      Serial.println("Tidak ada api");
    }
  }
}

void getAMG()
{
    amg.readPixels(pixels);
   Serial.print("[");
    for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
      x=pixels[i-1];
      AMG[i-1]=x;
      if ( x>40) {
      Serial.print("X");} else 
      {
         Serial.print(x);
         AMG[i-1]=x;
      }
      Serial.print(", ");
      if( i%8 == 0 ) Serial.println();
    }
    Serial.println("]");
    Serial.println();
}
void getCompas()
{
Serial.println("Kompas");
  Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
  Wire.write(ANGLE_8);                     //Sends the register we wish to start reading from
  Wire.endTransmission();

  // Request 5 bytes from the CMPS11
  // this will give us the 8 bit bearing,
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS11_ADDRESS, 5);

  while (Wire.available() < 5);       // Wait for all bytes to come back

  angle8 = Wire.read();               // Baca 3 Byte data kompas
  high_byte = Wire.read();
  low_byte = Wire.read();
  tmp = Wire.read();  // Abaikan 2 byte terahir (Data Roll & Pitch)
  tmp = Wire.read();

  angle16 = high_byte;                 // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += low_byte;

  heading = (angle16 / 10) + (angle16 % 10);
  Serial.print("Sudut: ");
  Serial.println(heading);  
  compass=heading;
}
void getLine()
{
   val = digitalRead (signalLine);
   
  if (val == 0)
  {
    Serial.println("Line Garis Putih");
    line=not(val);
  }
  else
  {
    Serial.println("Line Not");
    line=val;
  }
  delayMicroseconds(1);                           // Short delay before next loop
  Serial.println("");
}
void getAll()
{
    // PING
  getPing();
  //IR
  getIR();
  //UV
  getUV();
  //AMG 8833
  getAMG();
  //Kompas
  getCompas();
  // LINE SENSOR
  getLine();
}
void moveData()
{
  for(int i=0;i<12;i++)
  {
    DataSent[i]=Ping[i];
  }
    for(int i=12;i<76;i++)
  {
    DataSent[i]=AMG[i-12];
  }
    for(int i=76;i<78;i++)
  {
    DataSent[i]=IR[i-76];
  }
  DataSent[78]=line;
  DataSent[79]=UVtron;
  DataSent[81]=compass;
}
void isiData()
{
  for (int i=0;i<81;i++)
    {
    DataSent[i]=float(i+1)+0.19;
    }
    Serial.println("intialize");
    delay(2000);
}
void setup()
{
  Wire.begin();
  //pinMode sensor
  for (byte i=0 ; i<12;i++)
  {
  pinMode(signalPing[i], OUTPUT);
  }
    for (byte i=0 ; i<2;i++)
  {
  pinMode(signalIR[i], OUTPUT);
  }
      for (byte i=0 ; i<2;i++)
  {
  pinMode(signalUV[i], INPUT);
  }
  pinMode(signalMic, INPUT);
  pinMode(signalLine, INPUT);
  //Baud Rate
  Serial.begin(115200);
  bool status;
    
    // default settings
    status = amg.begin();
    if (!status) {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
        while (1);
    }
  isiData();
  //SOUND SENSOR
  
  /*while(sound != 0)
  {
    sound= digitalRead(signalMic);
  }*/
}
void loop() {
  while (1)
  {
  /*  //Serial.println("masih disini");
  received1=Serial.read();
  received2=Serial.read();
  received3=Serial.read();
  received4=Serial.read();
  received5=Serial.read();
  while (not((received1==99) or (received2==99) or (received3==99) or (received4==99) or (received5==99) ))
  {
    //Serial.println("masih diloop 99");
    received1=Serial.read();
  received2=Serial.read();
  received3=Serial.read();
  received4=Serial.read();
  received5=Serial.read(); 
  }
    delayMicroseconds(600);
  // START SENDING
   time2=millis(); 
    getAll();
     
    moveData();
  time1=millis();
  timer2=millis();
  for (int i=0;i<81;i++)
  {

    Serial.println('<'+String(DataSent[i],2)+'>');
    delayMicroseconds(1);
 
  }
  //Update semua sensor
   
  
  delta=time1-time2;
  timer1=millis();
  delta2=timer1-timer2;
  Serial.print("Runtime GetDAta: ");
  Serial.print(delta);
  Serial.println(" ms");
    Serial.print("Runtime serial: ");
  Serial.print(delta2);
  Serial.println(" ms");
  delay(2000);*/
measureDistance(A0);
}
}