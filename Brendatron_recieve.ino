#include <RH_ASK.h>
#include <SPI.h>
#include <Stepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//Radiohead reciever initialisation
RH_ASK receiver;

//PCA9685 initialisation
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
//Servo values
#define eSERVOMIN 130 //etronix servo min PMW
#define eSERVOMAX 450 //etronix servo max PMW
#define tSERVOMIN 160 //tower pro servo min PMW
#define tSERVOMAX 350 //tower pro servo max PMW
#define uSrvMin 10    //upper arm (s4V) servo min angle
#define uSrvMax 90   //upper arm (s4V) servo max angle

//store recieved data in struct:
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

//Servo pins
#define claw 5
#define wrist 7
#define foreArm 9
#define upperArm 11

//upper arm servo speed
#define upSpd 10

//analog input value buffers
int arm = 0;
int stepper = 0;
//transmition recieved boolean
bool rcv;
//wheel pwm debug
int pwm1dbg;
int pwm2dbg;
//signal loss counter
int lossctr;

//Wheel motor calculation variables
int pyin;
int pxin;
int pwml;
int pwmr;

//Wheel motor direction and PWM pins
#define wheell 2 //Left
#define wheelr 4 //Right
#define pwm1 3 //Left
#define pwm2 5 //Right

//Wheel motor on/off transistor signal pin
//#define wheelon 8

//stepper motor max speed (steps per second?)
#define stepSpeed 150
//Stepper motor direction pins
#define base1 6
#define base2 7

//Stepper motor initialisation
Stepper motor(768, base1, base2);
  
void setup() {
  Serial.begin(9600);
  Serial.println("ready");

  pinMode(LED_BUILTIN, OUTPUT);

  //if radiohead fails to initialise, flash onboard LED
  while(!receiver.init()) {
    Serial.println("Radiohead failed");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
  
  //Servo
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  pwm.setPWM(claw, 0, map(30, 0, 180, eSERVOMIN, eSERVOMAX)); //set start position to 30 degrees
  delay(10);
  pwm.setPWM(wrist, 0, map(180, 0, 180, tSERVOMIN, tSERVOMAX));
  delay(10);
  pwm.setPWM(foreArm, 0, map(10, 0, 180, tSERVOMIN, tSERVOMAX));
  delay(10);
  arm = 60; //used later, use this to set upper arm start pos
  pwm.setPWM(upperArm, 0, map(arm, 0, 180, eSERVOMIN, eSERVOMAX));
  
  //motor start positions and pin initialisations
  //Wheels
  pinMode(wheell, OUTPUT);
  pinMode(wheelr, OUTPUT);
  //pinMode(wheelon, OUTPUT);
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
  //delay(50);
  //digitalWrite(wheelon, HIGH);
  
  
  //stepper
  pinMode(base1, OUTPUT);
  pinMode(base2, OUTPUT);
  motor.setSpeed(20);

  //flash coherent message light twice
  digitalWrite(12, HIGH);
  delay(100);
  digitalWrite(12, LOW);
  delay(100);
  digitalWrite(12, HIGH);
  delay(100);
  digitalWrite(12, LOW);
  
  pinMode(9, OUTPUT); //low voltage indicator
}

void loop() {
  recieve();
  if(rcv) {
    process();
    //debug();
  }
  else {
    delay(5);
    lossctr++;
    //Serial.println(lossctr);
    if(lossctr > 100) {
      analogWrite(pwm1, 0);
      analogWrite(pwm2, 0);
    }
  }

  //voltage divider circuit for low voltage indicator - Vin = Vout*(R1+R2)/R2
  if((((float)map(analogRead(A0), 0, 1024, 0, 500)/100)*(4.7+4.7)/4.7) < 7) {
    digitalWrite(9, HIGH);
  } else {
    digitalWrite(9, LOW);
  }
}

void recieve() {
  rcv = false;
  
  digitalWrite(12, LOW);

  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  if(receiver.recv(buf, &buflen)) {
    memcpy(&data, buf, sizeof(data));

    rcv = true;
    lossctr = 0;
    digitalWrite(12, HIGH);
    //delay(5);
    
  }
}

void process() {
  //servos
  
  //Claw
  pwm.setPWM(claw, 0, map(data.s1V, 0, 250, tSERVOMIN, tSERVOMAX));
  //Wrist
  pwm.setPWM(wrist, 0, map(data.s2V, 0, 250, eSERVOMIN, eSERVOMAX));
  //Forearm
  pwm.setPWM(foreArm, 0, map(data.s3V, 0, 250, eSERVOMIN, eSERVOMAX));
  //Upper arm
  if(data.s4V < 127 && (arm - map(data.s4V, 0, 127, 1, upSpd)) > uSrvMin) {
    arm = arm - map(data.s4V, 127, 0, 1, upSpd);
  }
  else if(data.s4V > 128 && (arm + map(data.s4V, 128, 255, 1, upSpd)) < uSrvMax) {
    arm = arm + map(data.s4V, 128, 255, 1, upSpd);
  }
  pwm.setPWM(upperArm, 0, map(arm, 0, 180, eSERVOMIN, eSERVOMAX));

  //stepper
  
  if(data.stV < 127 /*&& (stepper - 10) > -200*/) {
    //stepper = stepper - map(data.stV, 0, 127, stepSpeed, 0);
    //motor.setSpeed(map(data.stV, 0, 127, stepSpeed, 0));
    motor.step(map(data.stV, 0, 127, -stepSpeed, 0));
  }
  else if(data.stV > 128 /*&& (stepper + 10) < 200*/) {
    //stepper = stepper + map(data.stV, 255, 127, stepSpeed, 0);
    //motor.setSpeed(map(data.stV, 128, 255, 0, stepSpeed));
    motor.step(map(data.stV, 255, 127, stepSpeed, 0));
  }

  //Wheels
//**************new wheels**************
  pxin = map(data.m1V, 0, 255, -255, 255);
  pyin = map(data.m2V, 0, 255, -255, 255);  

  if(abs(pyin) > abs(pxin)) {
    if(pyin > 0) {
      digitalWrite(wheell, HIGH);
      digitalWrite(wheelr, HIGH);
      Serial.print("both forward");
    } else {
      digitalWrite(wheell, LOW);
      digitalWrite(wheelr, LOW);
      Serial.print("both backward");  
    }
  } else {
    if(pxin < 0) {
      digitalWrite(wheell, HIGH);
      digitalWrite(wheelr, LOW);
      Serial.print("turning right");
    } else {
      digitalWrite(wheell, LOW);
      digitalWrite(wheelr, HIGH);
      Serial.print("turning left");
    }
  }

  if (pyin > 0) {
    if(pxin < 0) {
      Serial.print(" - left forward");
      if(pyin > abs(pxin)) { pwmr = pyin; } else { pwmr = abs(pxin); }
      pwml = abs(pyin + pxin);
    } else {
      Serial.print(" - right forward");
      pwmr = abs(pyin - pxin);
      if(pyin > pxin) { pwml = pyin; } else { pwml = pxin; }
    }
  } else {
    if(pxin < 0) {
      Serial.print(" - left backward");
      if(abs(pyin) > abs(pxin)) { pwml = abs(pyin); } else { pwml = abs(pxin); }
      pwmr = abs(pyin - pxin);
    } else {
      Serial.print(" - right backward");
      pwml = abs(pxin + pyin);
      if(abs(pyin) > pxin) { pwmr = abs(pyin); } else { pwmr = pxin; }
    }
  }
  
  analogWrite(pwm1, abs(pwml));
  analogWrite(pwm2, abs(pwmr));
  
  /*********************old wheels****************
  //left direction
  if(data.m1V>data.m2V) {
    digitalWrite(wheel1, HIGH);
  }
  else {
    digitalWrite(wheel1, LOW);
  }

  //right direction
  if(data.m1V > data.m2V && data.m1V > 255*0.75 ||
     data.m2V > data.m1V && data.m2V > 255*0.75) {
    digitalWrite(wheel2, HIGH);
  }
  else {
    digitalWrite(wheel2, LOW);
  }

  //PWM
  if(data.m1V != 255/2 && data.m2V == 255/2) {  //only steering
    analogWrite(pwm1, data.m1V);                //left = steering input
    analogWrite(pwm2, data.m1V);                //right = steering input
    pwm1dbg = data.m1V;
    pwm2dbg = data.m1V;
  }
  else if(data.m1V == 255/2 && data.m2V != 255/2) { //only linear
    analogWrite(pwm1, data.m2V);                    //left = linear input
    analogWrite(pwm2, data.m2V);                    //right = linear input
    pwm1dbg = data.m2V;
    pwm2dbg = data.m2V;
  }
  else if((data.m1V > 255/2 && data.m2V > 255/2) ||  //both high
     (data.m1V < 255/2 && data.m2V < 255/2)) {  //both low
    if(data.m1V > data.m2V) {                   //Linear is higher than Steering
      analogWrite(pwm2, data.m1V);              //Right = steering input
      pwm2dbg = data.m1V;
    }
    else {                                      //Steering is higher than linear
      analogWrite(pwm2, data.m2V);              //Right = linear input
      pwm2dbg = data.m2V;
    }
    analogWrite(pwm1, abs(data.m1V-data.m2V)); //Left = difference between inputs
    pwm1dbg = abs(data.m1V-data.m2V);
  }
  else if((data.m1V > 255/2 && data.m2V < 255/2) ||  //steering high, linear Low
     (data.m1V < 255/2 && data.m2V > 255/2)) {  //steering Low, linear High
    if(255/2 - data.m1V > data.m2V - 255/2) {   //Linear is higher than Steering
      analogWrite(pwm1, data.m1V);              //Left = steering input
      pwm1dbg = data.m1V;
    }
    else {                                      //Steering is higher than linear
      analogWrite(pwm1, data.m2V);              //Left = linear input
      pwm1dbg = data.m2V;
    }
    analogWrite(pwm2, abs(data.m1V-data.m2V)); //Right = difference between inputs
    pwm2dbg = abs(data.m1V-data.m2V);
  }
  else {
    analogWrite(pwm1, 0);
    analogWrite(pwm2, 0);
    pwm1dbg = 0;
    pwm2dbg = 0;
  }*/
}

void debug() {
  Serial.print("ctr: ");
  Serial.print(data.ctr);
  Serial.print(" -s1V: ");
  Serial.print(data.s1V);
  Serial.print(" -s2V: ");
  Serial.print(data.s2V);
  Serial.print(" -s3V: ");
  Serial.print(data.s3V);
  Serial.print(" -s4V: ");
  Serial.print(data.s4V);
  Serial.print(" -arm: ");
  Serial.print(arm);
  Serial.print(" -stV: ");
  Serial.print(data.stV);
  Serial.print(" -stepper: ");
  Serial.print(stepper);
  Serial.print(" -m1V: ");
  Serial.print(data.m1V);
  Serial.print(" -pwm1 debug: ");
  Serial.print(pwml);
  Serial.print(" -m2V: ");
  Serial.print(data.m2V);
  Serial.print(" -pwm2 debug: ");
  Serial.print(pwmr);
  //Serial.print(" -m1V+m2V: ");
  //Serial.print(data.m2V + data.m2V);
  //Serial.print(" -sqrt((m1V-m2V)^2): ");
  //Serial.print(((data.m1V-data.m2V)^2)^(1/2));

  Serial.println();
}
