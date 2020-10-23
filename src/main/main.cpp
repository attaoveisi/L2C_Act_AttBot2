#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>

#include <std_msgs/UInt16.h>

//#define not_use_track

ros::NodeHandle  nh;

//    The direction of the car's movement
//  ENA   ENB   IN1   IN2   IN3   IN4   Description  
//  HIGH  HIGH  HIGH  LOW   LOW   HIGH  Car is runing forward
//  HIGH  HIGH  LOW   HIGH  HIGH  LOW   Car is runing back
//  HIGH  HIGH  LOW   HIGH  LOW   HIGH  Car is turning left
//  HIGH  HIGH  HIGH  LOW   HIGH  LOW   Car is turning right
//  HIGH  HIGH  LOW   LOW   LOW   LOW   Car is stoped
//  HIGH  HIGH  HIGH  HIGH  HIGH  HIGH  Car is stoped
//  LOW   LOW   N/A   N/A   N/A   N/A   Car is stoped



//define L298n module IO Pin
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define LED 13

#define A8 3
#define A9 2
#define A10 0
#define A11 1
#define A12 12
#define A13 13 

#include <IRremote.h>

////////// IR REMOTE CODES //////////
#define FWD 16736925  // FORWARD
#define BWD 16754775  // BACK
#define LTR 16720605  // LEFT
#define RTR 16761405  // RIGHT
#define STP 16712445  // STOP
#define UNKNOWN_F 5316027     // FORWARD
#define UNKNOWN_B 2747854299  // BACK
#define UNKNOWN_L 1386468383  // LEFT
#define UNKNOWN_R 553536955   // RIGHT
#define UNKNOWN_S 3622325019  // STOP
#define KEY1 16738455
#define KEY2 16750695
#define KEY3 16756815
#define KEY4 16724175
#define KEY5 16718055
#define KEY6 16743045
#define KEY7 16716015
#define KEY8 16726215
#define KEY9 16734885
#define KEY0 16730805
#define KEY_STAR 16728765
#define KEY_HASH 16732845

#define RECV_PIN  4

unsigned char carSpeed = 250;
bool state = LOW;

long duration;
long distanceCm_new = 0, distanceCm_old = 0;
long ultraSound_center = 0, ultraSound_left = 0, ultraSound_right = 0;
long minEuclDistCm = 0;

IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long val;
unsigned long preMillis;

void forward(){ 
  digitalWrite(ENA,HIGH); //enable L298n A channel
  digitalWrite(ENB,HIGH); //enable L298n B channel
  digitalWrite(IN1,HIGH); //set IN1 hight level
  digitalWrite(IN2,LOW);  //set IN2 low level
  digitalWrite(IN3,LOW);  //set IN3 low level
  digitalWrite(IN4,HIGH); //set IN4 hight level
}

void back(){
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void left(){
  digitalWrite(ENA,carSpeed);
  digitalWrite(ENB,carSpeed);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH); 
}

void right(){
  digitalWrite(ENA,carSpeed);
  digitalWrite(ENB,carSpeed);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void stop(){
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW); 
}

void stateChange(){
  state = !state;
  digitalWrite(LED, state);;  
}

void track( const std_msgs::UInt16& cmd_msg){
  if(cmd_msg.data == 1){
     forward();
  }
  else if(cmd_msg.data == 2){
    right();
    }
  else if(cmd_msg.data == 3){
    left();
    }
  else{
    stop();
  } 
}

long getDistance(const int &trigPin,const int &echoPin, long &distanceCm_old) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distanceCm_old = distanceCm_new;
  distanceCm_new= duration*0.034/2.0f;
  if (abs((distanceCm_new-distanceCm_old) >= 100) || (distanceCm_new >= 600))
  {
    distanceCm_new = distanceCm_old;
  }
  return distanceCm_new;
}

long minEuclDist(const long &ultraSound_center, const long &ultraSound_left,const long &ultraSound_right){
  long minEuclDistCm = max(0,min(min(ultraSound_center, ultraSound_left),ultraSound_right));
  return minEuclDistCm;
}

ros::Subscriber<std_msgs::UInt16> sub("bangbang", track);

//before execute loop() function, 
//setup() function will execute first and only execute once
void setup() {
  Serial.begin(9600);//open serial and set the baudrate

  nh.initNode();
  nh.subscribe(sub);
  
  pinMode(IN1,OUTPUT);//before useing io pin, pin mode must be set first 
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  stop();
  irrecv.enableIRIn(); 
}

//Repeat execution
void loop() {

  //nh.spinOnce();
  //delay(4);

  ultraSound_center = getDistance(A8,A9,ultraSound_center); // Gets distance from the sensor and this function is repeatedly called while we are at the first example in order to print the lasest results from the distance sensor
  ultraSound_left = getDistance(A10,A11,ultraSound_left);
  ultraSound_right = getDistance(A12,A13,ultraSound_right);
  // Serial.println(ultraSound_left);
  // Serial.println(ultraSound_center);
  // Serial.println(ultraSound_right);
  //Serial.println("----");
  minEuclDistCm = minEuclDist(ultraSound_center, ultraSound_left, ultraSound_right);
  // Serial.println(minEuclDistCm);
  // Serial.println("----");
  // Serial.println("----");

  #ifdef not_use_track
    if(Serial.available())
    {
      char getstr = Serial.read();
      switch(getstr){
        case 'f': forward(); break;
        case 'b': back();   break;
        case 'l': left();   break;
        case 'r': right();  break;
        case 's': stop();   break;
        case 'a': stateChange(); break;
        default:  break;
      }
    }
    if (irrecv.decode(&results)){ 
      preMillis = millis();
      val = results.value;
      irrecv.resume();
      switch(val){
        case FWD: 
        case UNKNOWN_F: forward(); break;
        case BWD: 
        case UNKNOWN_B: back(); break;
        case LTR: 
        case UNKNOWN_L: left(); break;
        case RTR: 
        case UNKNOWN_R: right();break;
        case STP: 
        case UNKNOWN_S: stop(); break;
        default: break;
      }
    }
    else{
      if(millis() - preMillis > 500){
        stop();
        preMillis = millis();
      }
    }
  #endif
  forward();
  
//  if(minEuclDistCm < 10){
//    stop();
//  }
}
