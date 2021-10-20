#include <RH_ASK.h>
#include <SPI.h>

RH_ASK transmitter;

const int b1pin = 3;
const int b2pin = 4;
const int b3pin = 5;
const int b4pin = 6;
const int b5pin = 7;
const int b6pin = 8;

const int jLXpin = A0;
const int jLYpin = A1;
const int jRXpin = A2;
const int jRYpin = A3;

struct dataPackage {
  byte ctr;
  byte s1V;
  byte s2V;
  byte s3V;
  byte s4V;
  byte stV;
  byte m1V;
  byte m2V;
}data;

byte tx_buf[sizeof(data)] = {0};

//button control speeds
#define s1stp 10
#define s2stp 10
#define s3stp 10

//joystick dead zones
#define jLXZeroMax 570
#define jLXZeroMin 470
#define jLYZeroMax 570
#define jLYZeroMin 470
#define jRXZeroMax 570
#define jRXZeroMin 470
#define jRYZeroMax 570
#define jRYZeroMin 470

//joystick analog ranges
#define jLXMax 910
#define jLXMin 160
#define jLYMax 910
#define jLYMin 160
#define jRXMax 910
#define jRXMin 160
#define jRYMax 910
#define jRYMin 160

bool trns;
bool trns0;

void setup() {
  Serial.begin(9600);
  Serial.println("ready");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(2, OUTPUT); //low voltage indicator
  
  while(!transmitter.init()) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  
  pinMode(b1pin, INPUT);
  pinMode(b2pin, INPUT);
  pinMode(b3pin, INPUT);
  pinMode(b4pin, INPUT);
  pinMode(b5pin, INPUT);
  pinMode(b6pin, INPUT);

  data.ctr = 0;
  data.s1V = map(30, 0, 180, 0, 250); //set claw start position
  data.s2V = map(130, 0, 180, 0, 250); //set wrist start position
  data.s3V = map(30, 0, 180, 0, 250); //set fore arm
  //data.s4V = 100/*map(140, 0, 180, 0, 250)*/; //set upper arm

  delay(50);
}

void loop() {
  input();
  if(!trns && trns0) {
    for(int i = 0; i < 3; i++) {
      transmit();
      //delay(5);
    }
  }
  else if(trns || trns0) {
    transmit();
  }
  else {
    delay(50);
  }
  
  if((((float)map(analogRead(A5), 0, 1024, 0, 500)/100)*(4.7+4.7)/4.7) < 7) {
    digitalWrite(2, HIGH);
  } else {
    digitalWrite(2, LOW);
  }
}

void input() {
  
  trns = false;
  trns0 = false;
  
  if(digitalRead(b1pin) == HIGH && data.s1V < 250 - s1stp) {
    data.s1V = data.s1V + s1stp;
    trns = true;
  }
  else if(digitalRead(b2pin) == HIGH && data.s1V > 0 + s1stp) {
    data.s1V = data.s1V - s1stp;
    trns = true;
  }
  if(digitalRead(b3pin) == HIGH && data.s2V < 250 - s1stp) {
    data.s2V = data.s2V + s2stp;
    trns = true;
  }
  else if(digitalRead(b4pin) == HIGH && data.s2V > 0 + s1stp) {
    data.s2V = data.s2V - s2stp;
    trns = true;
  }
  if(digitalRead(b5pin) == HIGH && data.s3V < 250 - s1stp) {
    data.s3V = data.s3V + s3stp;
    trns = true;
  }
  else if(digitalRead(b6pin) == HIGH && data.s3V > 0 + s1stp) {
    data.s3V = data.s3V - s3stp;
    trns = true;
  }
  if(analogRead(jLXpin) < jLXZeroMin  || analogRead(jLXpin) > jLXZeroMax) {
    data.s4V = map(analogRead(jLXpin), jLXMin, jLXMax, 0, 255);
    trns = true;
  }
  else if(data.s4V < map(jLXZeroMin, jLXMin, jLXMax, 0, 255) || data.s4V > map(jLXZeroMax, jLXMin, jLXMax, 0, 255)) {
    data.s4V = 255/2;
    trns0 = true;
  }
  if(analogRead(jLYpin) < jLYZeroMin  || analogRead(jLYpin) > jLYZeroMax) {
    data.stV = map(analogRead(jLYpin), jLYMin, jLYMax, 0, 255);
    trns = true;
  }
  else if(data.stV < map(jLYZeroMin, jLYMin, jLYMax, 0, 255) || data.stV > map(jLYZeroMax, jLYMin, jLYMax, 0, 255)) {
    data.stV = 255/2;
    trns0 = true;
  }
  if(analogRead(jRXpin) < jRXZeroMin  || analogRead(jRXpin) > jRXZeroMax) {
    data.m1V = map(analogRead(jRXpin), jRXMin, jRXMax, 0, 255);
    trns = true;
  }
  else if(data.m1V < map(jRXZeroMin, jRXMin, jRXMax, 0, 255) || data.m1V > map(jRXZeroMax, jRXMin, jRXMax, 0, 255)) {
    data.m1V = 255/2;
    trns0 = true;
  }
  if(analogRead(jRYpin) < jRYZeroMin  || analogRead(jRYpin) > jRYZeroMax) {
    data.m2V = map(analogRead(jRYpin), jRYMin, jRYMax, 0, 255);
    trns = true;
  }
  else if(data.m2V < map(jRYZeroMin, jRYMin, jRYMax, 0, 255) || data.m2V > map(jRYZeroMax, jRYMin, jRYMax, 0, 255)) {
    data.m2V = 255/2;
    trns0 = true;
  }
}

void transmit() {
  /*
  Serial.print(": jLX = ");
  Serial.print(analogRead(jLXpin));
  Serial.print(" - jLY = ");
  Serial.print(analogRead(jLYpin));
  Serial.print(" - jRX = ");
  Serial.print(analogRead(jRXpin));
  Serial.print(" - jRY = ");
  Serial.println(analogRead(jRYpin));
  */
  data.ctr++;
  Serial.print(data.ctr);
  Serial.print(": s1V = ");
  Serial.print(data.s1V);
  Serial.print(" - s2V = ");
  Serial.print(data.s2V);
  Serial.print(" - s3V = ");
  Serial.print(data.s3V);
  Serial.print(" - s4V = ");
  Serial.print(data.s4V);
  Serial.print(" - stV = ");
  Serial.print(data.stV);
  Serial.print(" - m1V = ");
  Serial.print(data.m1V);
  Serial.print(" - m2V = ");
  Serial.println(data.m2V);
  
  memcpy(tx_buf, &data, sizeof(data));
  byte zize = sizeof(data);
  transmitter.send((uint8_t *)tx_buf, zize);
  transmitter.waitPacketSent();
  
  //delay(20);
}
