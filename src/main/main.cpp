#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <IRremote.h>
#include <std_msgs/UInt16.h>

//#define not_use_track

ros::NodeHandle nh;

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
#define ENA 10
#define IN1 2
#define IN2 4
#define ENA_WA 9
#define IN5 8
#define IN6 7

#define A8 13
#define A9 12
#define A10 A2
#define A11 6
#define A12 5
#define A13 3

#define buzzer A1 //buzzer to arduino pin 9

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

#define RECV_PIN 11

int carSpeedfb = 255; // car speed for forward and backward
int carSpeedlr = 255; // car speed for left and right
int carWA = 255; // wheel angle 

float duration;
float distanceCm_new = 0.0, distanceCm_old = 0.0;
float ultraSound_center = 0.0, ultraSound_left = 0.0, ultraSound_right = 0.0;
float minEuclDistCm = 0.0;

IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long val;
unsigned long preMillis;

bool state = LOW;
#define LED A3

void forward(){ 
  analogWrite(ENA, carSpeedfb); //enable L298n A channel
  digitalWrite(IN1,HIGH); //set IN1 hight level
  digitalWrite(IN2,LOW);  //set IN2 low level
}

void back(){
  analogWrite(ENA, carSpeedfb);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH); 
}

void left(){
  analogWrite(ENA_WA,carWA); //enable L298n A channel
  digitalWrite(IN5,LOW); //set IN3 hight level
  digitalWrite(IN6,HIGH);  //set IN4 low level  
  delay(100);
  digitalWrite(ENA,HIGH); //enable L298n A channel
  digitalWrite(IN1,HIGH); //set IN1 hight level
  digitalWrite(IN2,LOW);  //set IN2 low level
  
}

void right(){
  analogWrite(ENA_WA,carWA); //enable L298n A channel
  digitalWrite(IN5,HIGH); //set IN3 hight level
  digitalWrite(IN6,LOW);  //set IN4 low level
  delay(100);
  digitalWrite(ENA,HIGH); //enable L298n A channel
  digitalWrite(IN1,HIGH); //set IN1 hight level
  digitalWrite(IN2,LOW);  //set IN2 low level
}

void stop(){
  digitalWrite(ENA, LOW);
  digitalWrite(ENA_WA, LOW); 
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

float getDistance(const int &trigPin,const int &echoPin, float &distanceCm_old) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distanceCm_old = distanceCm_new;
  distanceCm_new = duration*0.034/2.0;
  if (abs((distanceCm_new-distanceCm_old) >= 100.0) || (distanceCm_new >= 600.0))
  {
    distanceCm_new = distanceCm_old;
  }
  return distanceCm_new;
}

float minEuclDist(const long &ultraSound_left,const long &ultraSound_right){
  float minEuclDistCm = max(0,min(ultraSound_right, ultraSound_left));
  return minEuclDistCm;
}

void distance(){
  ultraSound_center = getDistance(A8,A9,ultraSound_center); // Gets distance from the sensor and this function is repeatedly called while we are at the first example in order to print the lasest results from the distance sensor
  ultraSound_left = getDistance(A10,A11,ultraSound_left);
  ultraSound_right = getDistance(A12,A13,ultraSound_right);

  minEuclDistCm = minEuclDist(ultraSound_left, ultraSound_right);

  // Serial.println(minEuclDistCm);
  // Serial.println(ultraSound_center);
  // Serial.println("---------");
}

void stateChange(){
  state = !state;
  digitalWrite(LED, state);;  
}

ros::Subscriber<std_msgs::UInt16> sub("bangbang", track);

//before execute loop() function, 
//setup() function will execute first and only execute once
void setup() {
  // Serial.begin(9600);//open serial and set the baudrate

  nh.initNode();
  nh.subscribe(sub);
  
  pinMode(IN1,OUTPUT); //before using io pin, pin mode must be set first 
  pinMode(IN2,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENA_WA,OUTPUT);
  pinMode(IN5,OUTPUT);
  pinMode(IN6,OUTPUT);
  pinMode(buzzer, OUTPUT); // Set buzzer - pin as an output
  pinMode(A8, OUTPUT);
  pinMode(A10, OUTPUT);
  pinMode(A12, OUTPUT);
  pinMode(A9, INPUT);
  pinMode(A11, INPUT);
  pinMode(A13, INPUT);

  noTone(buzzer);     // Stop sound...
  stop();
  
  //irrecv.enableIRIn(); 
}

//Repeat execution
void loop() {

  nh.spinOnce();
  delay(4);

  distance();

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
  
 if ((ultraSound_left < 30.0 && ultraSound_left > 5.0) || (ultraSound_right < 30.0 && ultraSound_right > 5.0)){
  stop();
  while ((ultraSound_left < 30.0 && ultraSound_left > 5.0) || (ultraSound_right < 30.0 && ultraSound_right > 5.0)){
    tone(buzzer, 1000); // Send 300 Hz sound signal...
    delay(1000);        // ...for 1 sec
    noTone(buzzer);     // Stop sound...
    back();
    delay(250);        // ...for .25 sec
    distance();
    if ((ultraSound_left >= 30.0) || (ultraSound_right >= 30.0)){
      break;
    }
    stop();
    delay(500);        // ...for 0.5 sec
  }
 }else if(ultraSound_center < 30.0 && ultraSound_center > 5.0){
  stop();
  while (ultraSound_center < 30.0 && ultraSound_center > 5.0){
    tone(buzzer, 300); // Send 500 Hz sound signal...
    delay(1000);        // ...for 1 sec
    noTone(buzzer);     // Stop sound...
    forward();
    delay(250);        // ...for .25 sec
    distance();
    if (ultraSound_center >= 30.0){
      break;
    }
    stop();
    delay(500);        // ...for 0.5 sec
  }
 }

}
