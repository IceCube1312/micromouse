const int irpinl =A0;
const int irpinr=A2;
const int irpinf = A1;
const int STBY = 8; //Standby pin of the driver will be connected to 8, will control the stop and start of motor
const int leftspeed = 3;
const int leftdir1 = 2;
const int leftdir2 = 4;
const int rightspeed = 9;
const int rightdir1 = 7;
const int rightdir2 = 8;
float Kp = 100; //Steering strength
float Kd = 15; //dampening
float Ki = 0;
int base_speed = 100;//(0-255 on the analogWrite would mean 0-5V)
int max_speed = 180;
int turning_speed = 100;
int last_error = 0;
void setup(){
  pinMode(irpinf,INPUT);
  pinMode(irpinl,INPUT);
  pinMode(irpinr,INPUT);
  pinMode(STBY,OUTPUT);
  pinMode(leftspeed,OUTPUT);
  pinMode(leftdir1,OUTPUT);
  pinMode(leftdir2,OUTPUT);
  pinMode(rightspeed,OUTPUT);
  pinMode(rightdir1,OUTPUT);
  pinMode(rightdir2,OUTPUT);
  digitalWrite(STBY,HIGH);
  Serial.begin(9600);  
  delay(2000); //Some gap after it is turned on

}
int readavg(int input_pin){
  sum =0;
  for(int i =1;i<=10;i++){
    sum+=analogRead(input_pin);
  }
  return sum/10;
}
bool API_wallFront(){
  int F = analogRead(irpinf);
  if(F>500){
    return false;
  }
  else{
    return true;
  }
}
bool API_wallLeft(){
  int L = analogRead(irpinl);
  if(L>500){
    return false;
  }
  else{
    return true;
  }
}
bool API_wallRight(){
  int R = analogRead(irpinr);
  if(R>500){
    return false;
  }
  else{
    return true;
  }
}
void setleftmotor(int speed,bool sense_of_rot){
  analogWrite(leftspeed,speed);
  if(sense_of_rot){
    digitalWrite(leftdir1,HIGH);
    digitalWrite(leftdir2,LOW);
  }
  else{
    digitalWrite(leftdir1,LOW);
    digitalWrite(leftdir2,HIGH);
  }
}
void setrightmotor(int speed,bool sense_of_rot){
  analogWrite(rightspeed,speed);
  if(sense_of_rot){
    digitalWrite(rightdir1,HIGH);
    digitalWrite(rightdir2,LOW);
  }
  else{
    digitalWrite(rightdir1,LOW);
    digitalWrite(rightdir2,HIGH);
  }
}
void API_turnRight(){
  setrightmotor(100,false); // The Boolean directions need to be tested
  setleftmotor(100,true);
}
void API_turnLeft(){
  setleftmotor(100,false); // The Boolean directions need to be tested
  setrightmotor(100,true);
}
