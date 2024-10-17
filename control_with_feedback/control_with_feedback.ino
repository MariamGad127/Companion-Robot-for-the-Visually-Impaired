
/********************ENCODERS********************/
//right encoder
#define ChannelA 13
#define ChannelB 4
//left encoder
#define ChannelC A5
#define ChannelD A0

typedef struct {
  signed int counter_R;
  signed int CurrentState_R;
  signed int LastState_R;
  signed int previousTick_R;
  signed int currentTick_R;

  int counter_L;
  int CurrentState_L;
  int LastState_L;
  int previousTick_L;
  int currentTick_L;

} Encoders_t;

Encoders_t Encoders;

int cell_is_done;
/********************MOTORS********************/
// Left motor
#define ENA  11
#define IN1  12
#define IN2  10

// Right motor
#define ENB  9
#define IN3  8
#define IN4  7

#define Speed  70
#define Offset 20



void setup() {
  Serial.begin (9600);

  // Rotary encoder
  pinMode (ChannelA, INPUT);
  pinMode (ChannelB, INPUT);
  pinMode (ChannelC, INPUT);
  pinMode (ChannelD, INPUT);


  // Encoder initial state
  Encoders.LastState_R = digitalRead(ChannelA);
  Encoders.LastState_L = digitalRead(ChannelC);

  Encoders = readEncoders();
  Encoders.currentTick_R = Encoders.counter_R;
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
  Encoders = readEncoders();
  Encoders.currentTick_R = Encoders.counter_R;
  Encoders.currentTick_L = Encoders.counter_L;

  // Move for one cell in each direction
  forward_FB();
  delay(500);
    forward_FB();
  delay(500);
  backward_FB();
  delay(500);
  right_FB();
  SStop();
  delay(500);
  left_FB();
    SStop();
  delay(500);

  Encoders = readEncoders();
  Encoders.previousTick_R = Encoders.currentTick_R;
  Encoders.previousTick_L = Encoders.currentTick_L;

}


Encoders_t readEncoders() {

  // Read right encoder
  Encoders.CurrentState_R = digitalRead(ChannelA);
  // Change in state --> new pulse
  if (Encoders.CurrentState_R != Encoders.LastState_R) {
    // ChannelB different from current state --> clockwise
    if (digitalRead(ChannelB) != Encoders.CurrentState_R) {
      Encoders.counter_R --;
    }
    else {
      Encoders.counter_R ++;
    }
    Serial.print(" Position_R: ");
    Serial.print(Encoders.counter_R);
  }
  Encoders.LastState_R = Encoders.CurrentState_R;


  // Read left encoder
  Encoders.CurrentState_L = digitalRead(ChannelC);
  // Change in state --> new pulse
  if (Encoders.CurrentState_L != Encoders.LastState_L) {
    // ChannelB different from current state --> clockwise
    if (digitalRead(ChannelD) != Encoders.CurrentState_L) {
      Encoders.counter_L --;
    }
    else {
      Encoders.counter_L ++;
    }
    Serial.print("    Position_L: ");
    Serial.println(Encoders.counter_L);
  }
  Encoders.LastState_L = Encoders.CurrentState_L;

  return Encoders;

}


/************************CONTROL WITH FEEDBACK************************/
void forward_FB() {
  Encoders = readEncoders();
  Encoders.previousTick_R = Encoders.counter_R;
  Encoders.previousTick_L = Encoders.counter_L;
  // Serial.println((String)"  previousTick_R: " + Encoders.previousTick_R + (String)"  previousTick_L: " + Encoders.previousTick_L);

  Serial.write('F');

  while ((Encoders.currentTick_L - Encoders.previousTick_L <= 5) && (Encoders.currentTick_R - Encoders.previousTick_R <= 5)) {
    if (Encoders.currentTick_L - Encoders.previousTick_L <= 5) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, Speed);
    }
    if (Encoders.currentTick_R - Encoders.previousTick_R <= 5) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, Speed + Offset);
    }

    Encoders = readEncoders();
    Encoders.currentTick_R = Encoders.counter_R;
    Encoders.currentTick_L = Encoders.counter_L;
    // Serial.println((String)"  currentTick_R: " + Encoders.currentTick_R + (String)"  currentTick_L: " + Encoders.currentTick_L);
  }
  cell_is_done = 1;
}

void backward_FB() {
  Encoders = readEncoders();
  Encoders.previousTick_R = Encoders.counter_R;
  Encoders.previousTick_L = Encoders.counter_L;
  //Serial.println((String)"  previousTick_R: " + Encoders.previousTick_R + (String)"  previousTick_L: " + Encoders.previousTick_L);

  Serial.write('B');

  while ((Encoders.previousTick_L - Encoders.currentTick_L <= 10) && (Encoders.previousTick_R - Encoders.currentTick_R <= 10)) {
    if (Encoders.previousTick_L - Encoders.currentTick_L <= 10) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, Speed);
    }
    if (Encoders.previousTick_R - Encoders.currentTick_R <= 10) {
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


void right_FB() {
  Encoders = readEncoders();
  Encoders.previousTick_R = Encoders.counter_R;
  Encoders.previousTick_L = Encoders.counter_L;
  //Serial.println((String)"  previousTick_R: " + Encoders.previousTick_R + (String)"  previousTick_L: " + Encoders.previousTick_L);

  Serial.write('R');

  while ((Encoders.currentTick_L - Encoders.previousTick_L < 7) && (Encoders.currentTick_R - Encoders.previousTick_R > -7)) {
    if (Encoders.currentTick_L - Encoders.previousTick_L < 7) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, Speed);
    }
    if (Encoders.currentTick_R - Encoders.previousTick_R > -7) {
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



void left_FB() {
  Encoders = readEncoders();
  Encoders.previousTick_R = Encoders.counter_R;
  Encoders.previousTick_L = Encoders.counter_L;
  // Serial.println((String)"  previousTick_R: " + Encoders.previousTick_R + (String)"  previousTick_L: " + Encoders.previousTick_L);
  
  Serial.write('L');

  while ((Encoders.currentTick_L - Encoders.previousTick_L > -7) && (Encoders.currentTick_R - Encoders.previousTick_R < 7)) {
    if (Encoders.currentTick_L - Encoders.previousTick_L > -7) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, Speed);
    }
    if (Encoders.currentTick_R - Encoders.previousTick_R < 7) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, Speed + Offset);
    }

    Encoders = readEncoders();
    Encoders.currentTick_R = Encoders.counter_R;
    Encoders.currentTick_L = Encoders.counter_L;
    // Serial.println((String)"  currentTick_R: " + Encoders.currentTick_R + (String)"  currentTick_L: " + Encoders.currentTick_L);
  }
}

void SStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}