/********************ULTRASONIC********************/
typedef struct {
  int Left;
  int Right;
  int Front;
  int Top;
} Distances_t;

#include <Ultrasonic.h>
Ultrasonic uTop(A3, A4);
Ultrasonic uRight(6, 5);
Ultrasonic uFront(A1, A2);
Ultrasonic uLeft(2, 3);

int LeftDistance;
int RightDistance;
int FrontDistance;
int TopDistance;

int threshold = 20;
int topThreshold = 60;


/********************MOTORS********************/
// Left motor
#define ENA 11
#define IN1 12
#define IN2 10

// Right motor
#define ENB 9
#define IN3 8
#define IN4 7

#define Speed 60
#define Offset 15

#define delatTime 5

typedef enum {
  forward,
  right,
  left,
  back_then_right,
  right_forward_left,
  right_two_forward_right,
  left_two_forward_left,
  stopp,

} Directions_t;

Directions_t Direction_map;


/********************ENCODERS********************/
/*
//right encoder
#define ChannelA 13
#define ChannelB 4
*/
//left encoder
#define ChannelC A5
#define ChannelD A0

typedef struct {

  int LeftDistance;
int RightDistance;
int FrontDistance;
int TopDistance;

  int counter_R;
  int CurrentState_R;
  int LastState_R;
  int previousTick_R;
  int currentTick_R;

  int counter_L;
  int CurrentState_L;
  int LastState_L;
  int previousTick_L;
  int currentTick_L;

} Encoders_t;

Encoders_t Encoders;


/********************SOUND********************/
#include "mp3tf16p.h"
MP3Player mp3(4, 13);

#define forward_sound 3
#define backward_sound 4
#define left_sound 1
#define right_sound 2


/********************FUNCTION PROTOTYPES********************/
Encoders_t readEncoders();
Directions_t obstacleAvoider();
void printDirection();
void forward_FB();
void backward_FB();
void right_FB();
void left_FB();
void SStop();


void setup() {
  Serial.begin(9600);

  mp3.initialize();

  // Rotary encoder
  /*
  pinMode(ChannelA, INPUT);
  pinMode(ChannelB, INPUT);
  */
  pinMode(ChannelC, INPUT);
  pinMode(ChannelD, INPUT);

  // Encoder initial state
  //Encoders.LastState_R = digitalRead(ChannelA);
  Encoders.LastState_L = digitalRead(ChannelC);

  Encoders = readEncoders();
  //Encoders.currentTick_R = Encoders.counter_R;
  Encoders.currentTick_L = Encoders.counter_L;


  // Motors
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}


void loop() {

      // Read distance readings from all sides
  LeftDistance = uLeft.read();
  RightDistance = uRight.read();
  FrontDistance = uFront.read();
  TopDistance = uTop.read();
  
  Serial.println((String)"Front: " + FrontDistance + (String)"  Left: " + LeftDistance + (String)"  Right: " + RightDistance + (String)"  Top: " + TopDistance);
  
  Encoders = readEncoders();
  //Encoders.currentTick_R = Encoders.counter_R;
  Encoders.currentTick_L = Encoders.counter_L;

  Direction_map = obstacleAvoider();

  switch (Direction_map) {
    case forward:
      forward_FB();
      SStop();
      delay(delatTime);
      break;

    case back_then_right:
      backward_FB();
      right_FB();
      SStop();
      delay(delatTime);
      break;

    case right:
      right_FB();
      SStop();
      delay(delatTime);
      break;

    case left:
      left_FB();
      SStop();
      delay(delatTime);
      break;

    default:
      SStop();
      break;
  }

  Encoders = readEncoders();
  //Encoders.previousTick_R = Encoders.currentTick_R;
  Encoders.previousTick_L = Encoders.currentTick_L;
}


Directions_t obstacleAvoider() {
  // Read distance readings from all sides
  LeftDistance = uLeft.read();
  RightDistance = uRight.read();
  FrontDistance = uFront.read();
  TopDistance = uTop.read();

  //Serial.println((String)"Front: " + FrontDistance + (String)"  Left: " + LeftDistance + (String)"  Right: " + RightDistance + (String)"  Top: " + TopDistance);

  // Top obstacles, move backward then right
  if (TopDistance < 60) {
    return back_then_right;
  }

  // Obstacle in the front, turn right
  else if ((FrontDistance < threshold) && (RightDistance > threshold)) {
    return right;
  }
  // Front and right cells are occupied, turn left
  else if ((FrontDistance < threshold) && (RightDistance < threshold) && (LeftDistance > threshold)) {
    return left;
  } else {
    return forward;
  }
}

Encoders_t readEncoders() {
  /*
  // Read right encoder
  Encoders.CurrentState_R = digitalRead(ChannelA);
  // Change in state --> new pulse
  if (Encoders.CurrentState_R != Encoders.LastState_R) {
    // ChannelB different from current state --> clockwise
    if (digitalRead(ChannelB) != Encoders.CurrentState_R) {
      Encoders.counter_R--;
    } else {
      Encoders.counter_R++;
    }
    //Serial.print(" Position_R: ");
    //Serial.print(Encoders.counter_R);
  }
  Encoders.LastState_R = Encoders.CurrentState_R;
  */

  // Read left encoder
  Encoders.CurrentState_L = digitalRead(ChannelC);
  // Change in state --> new pulse
  if (Encoders.CurrentState_L != Encoders.LastState_L) {
    // ChannelB different from current state --> clockwise
    if (digitalRead(ChannelD) != Encoders.CurrentState_L) {
      Encoders.counter_L--;
    } else {
      Encoders.counter_L++;
    }
    //Serial.print("    Position_L: ");
    //Serial.println(Encoders.counter_L);
  }
  Encoders.LastState_L = Encoders.CurrentState_L;

  return Encoders;
}

/************************CONTROL WITH FEEDBACK************************/
void forward_FB() {
  Encoders = readEncoders();
  //Encoders.previousTick_R = Encoders.counter_R;
  Encoders.previousTick_L = Encoders.counter_L;
  // Serial.println((String)"  previousTick_R: " + Encoders.previousTick_R + (String)"  previousTick_L: " + Encoders.previousTick_L);

  mp3.playTrackNumber(forward_sound, 30);


  while ((Encoders.currentTick_L - Encoders.previousTick_L <= 5)) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, Speed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, Speed + Offset);

    Encoders = readEncoders();
    //Encoders.currentTick_R = Encoders.counter_R;
    Encoders.currentTick_L = Encoders.counter_L;
    // Serial.println((String)"  currentTick_R: " + Encoders.currentTick_R + (String)"  currentTick_L: " + Encoders.currentTick_L);
  }
}

void backward_FB() {
  Encoders = readEncoders();
  //Encoders.previousTick_R = Encoders.counter_R;
  Encoders.previousTick_L = Encoders.counter_L;
  //Serial.println((String)"  previousTick_R: " + Encoders.previousTick_R + (String)"  previousTick_L: " + Encoders.previousTick_L);

  mp3.playTrackNumber(backward_sound, 30);

  while ((Encoders.previousTick_L - Encoders.currentTick_L <= 5)) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, Speed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, Speed + Offset);


    Encoders = readEncoders();
    //Encoders.currentTick_R = Encoders.counter_R;
    Encoders.currentTick_L = Encoders.counter_L;
    //Serial.println((String)"  currentTick_R: " + Encoders.currentTick_R + (String)"  currentTick_L: " + Encoders.currentTick_L);
  }
}

/*
void right_FB() {
  Encoders = readEncoders();
  Encoders.previousTick_R = Encoders.counter_R;
  Encoders.previousTick_L = Encoders.counter_L;
  //Serial.println((String)"  previousTick_R: " + Encoders.previousTick_R + (String)"  previousTick_L: " + Encoders.previousTick_L);

  Serial.write('R');

  while ((Encoders.currentTick_L - Encoders.previousTick_L <= 7) && (Encoders.currentTick_R - Encoders.previousTick_R >= -7)) {
    if (Encoders.currentTick_L - Encoders.previousTick_L <= 7) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, Speed);
    }
    if (Encoders.currentTick_R - Encoders.previousTick_R >= -7) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, Speed + Offset);
    }

    Encoders = readEncoders();
    Encoders.currentTick_R = Encoders.counter_R;
    Encoders.currentTick_L = Encoders.counter_L;
    //Serial.println((String)"  currentTick_R: " + Encoders.currentTick_R + (String)"  currentTick_L: " + Encoders.currentTick_L);
  }
}
*/

void right_FB() {

  mp3.playTrackNumber(right_sound, 30);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, Speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, Speed + Offset);
  delay(500);
}

void left_FB() {
  mp3.playTrackNumber(left_sound, 30);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, Speed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, Speed + Offset);
  delay(500);
}

void SStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
