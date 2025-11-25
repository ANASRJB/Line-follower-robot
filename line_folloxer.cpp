#include <QTRSensors.h>
#include <Arduino.h>

const uint8_t SensorCount = 12;
unsigned char sensorPins[] = {25,26,27,14}; // Broches des capteurs
unsigned int s[SensorCount];
QTRSensorsRC qtr(sensorPins, SensorCount); // Déclaration des broches des capteurs

int position;
//motor
int rightF=32;
int rightR=33;
int leftF=4;
int leftR=15;
int n=0;
//PID
int right_speed,left_speed; 
float kp=0.095,kd=0.8 ,ki=0,P,D,I;
float PIDvalue,lasterror,error,error1,lasterror1;
int left_base=160,right_base=160;
unsigned long current_time,last_time,lunch_time;
int inter=36;
int led1=2 ;
int c1;
int c2;
int c3;
int c4;
void setup() {
  
Serial.begin(9600);
   //moteurs
  pinMode(rightF,OUTPUT);
  pinMode(leftF,OUTPUT);
  pinMode(rightR,OUTPUT);
  pinMode(leftR,OUTPUT);
  pinMode(inter,INPUT);
  digitalWrite(led1, HIGH);// turn on Arduino's LED to indicate we are in calibration mode
   for (int i = 0 ; i < 200; i++)
  {
    qtr.calibrate();
    tst=1;
    delay(10);
  }
  digitalWrite(led1, LOW); 
   current_time=millis();
   int x=0;
  while(1){
     x=digitalRead(inter);
     Serial.println(x);
     if(x==HIGH){
        lunch_time=millis();
         if(lunch_time - current_time>400)
           break;}
  }
  stp(500);
 last_time=millis();

 forward(200);

   
}
 

void loop() {
 position= qtr.readLine(s);
 int c1=s[0];
 int c2=s[1];
 int c3=s[2];
 int c4=s[3];
 int somme=c1+c2+c3+c4;
 int current_time=millis(); 
  //test PID
 //while(1){
 //PID;
 //forwardPID();}

  if(n==0 && c2+c1>1500 && current_time>500){
    forward(30);
    last_time=current_time;
    n=1;
  }
  /*//if(n==1 && c1+c2>1500 && current_time-last_time>400){
    stp(100);
        last_time=current_time;
      forward(30);
    n=2;
  }*/
 
}
//functions
void PID(){
  position = qtr.readLine(s);
  error=position-5500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(255,max(left_speed,0));
  right_speed=min(255,max(right_speed,0));

}
void pidforcee(){
  position = qtr.readLine(s);
  if(position<2000){left(1);}
  else if (position>13000){right(1);} 
 else{ error=position-7500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0)); } 
}
void PID_noir(){
  position = qtr.readLine(s,QTR_EMITTERS_ON,1,true);
  error=position-5500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  left_speed=left_base-PIDvalue;
  right_speed=right_base+PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
}
void PID_special(){
  bool v=false;
  position = qtr.readLine(s,QTR_EMITTERS_ON,0,v);
  error=position-5500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void forwardPID(){
  
  analogWrite(rightF,right_speed);
   analogWrite(leftF,left_speed);
  analogWrite(rightR,0);
  analogWrite(leftR,0);

}
void forward(int x){
  
  analogWrite(rightF,160);
   analogWrite(leftF,160);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}

void left(int x){
  
  analogWrite(rightF,100);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}

void right(int x){
  
  analogWrite(rightF,0);
  analogWrite(leftF,160);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void left_fblstou(int x){
  
  analogWrite(rightF,140);
  analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,140);
  delay(x);
}
void right_fblstou(int x){
  
  analogWrite(rightF,0);
  analogWrite(leftF,140);
  analogWrite(rightR,140);
  analogWrite(leftR,0);
  delay(x);
}
void stp(long int x){
  
  analogWrite(rightF,0);
   analogWrite(leftF,0);
  analogWrite(rightR,0);
  analogWrite(leftR,0);
  delay(x);
}
void back(int x){
   analogWrite(rightF,0);
   analogWrite(leftF,0);
  analogWrite(rightR,100);
  analogWrite(leftR,100);
  delay(x);
}
void PID_speed(){
  position = qtr.readLine(s);
  error=position-7000;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void PID_chlot(){
  position = qtr.readLineM(s);
  error=position-3500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(255,max(left_speed,0));
  right_speed=min(255,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void PID_noirE(){
  position = qtr.readLineM(s,QTR_EMITTERS_ON,1,false);
  error=position-3500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  left_speed=left_base-PIDvalue;
  right_speed=right_base+PIDvalue;
  left_speed=min(250,max(left_speed,0));
  right_speed=min(250,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}
void PID_specialE(){
  bool v=false;
  position = qtr.readLine(s,QTR_EMITTERS_ON,0,v);
  error=position-5500;
  P = error;
  D = error-lasterror;
  I=error+I;
  PIDvalue =(kp*P)+(kd*D)+(ki*I);
  lasterror = error;
  right_speed=right_base+PIDvalue;
  left_speed=left_base-PIDvalue;
  left_speed=min(100,max(left_speed,0));
  right_speed=min(100,max(right_speed,0));
  //delay(200);
  //Serial.println(error);
}

 