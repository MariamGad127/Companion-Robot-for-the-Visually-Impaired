
/*********************ENCODERS*********************/
#define R_ChannelA 13
#define R_ChannelB 4
#define L_ChannelA A0
#define L_ChannelB A5

long R_Counter, L_Counter;
int R_CurrentState, L_CurrentState;
int R_LastState, L_LastState;

long R_ticks_prev, L_ticks_prev;

float ticks_per_revolution = 30.0;

long time_prev, time_current, time_interval;

/*********************MOTORS*********************/
// Left motor
#define ENA  11
#define IN1  12
#define IN2  10

// Rigt motor
#define ENB  9
#define IN3  8
#define IN4  7


/*********************PID*********************/
float Kp = 25.0;
float Ki = 0.0;
float Kd = 0.0;

float SetPoint = 2.5;  // desired velocity
float R_Error, R_PreviousError = 0.0;
float L_Error, L_PreviousError = 0.0;

typedef struct{
int R_PID;
int L_PID;
} Output_PID_t;

Output_PID_t Output;

void setup() {
  pinMode (R_ChannelA, INPUT);
  pinMode (R_ChannelB, INPUT);
  pinMode (L_ChannelA, INPUT);
  pinMode (L_ChannelB, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin (9600);
  // Initial state
  R_LastState = digitalRead(R_ChannelA);
  L_LastState = digitalRead(L_ChannelA);
  // time initial state set right after reading the encoder
  time_prev = millis();
}

void loop() {
  R_CurrentState = digitalRead(R_ChannelA);
  L_CurrentState = digitalRead(L_ChannelA);

  // Change in state --> new pulse
  if (R_CurrentState != R_LastState) {
    // ChannelB different from current state --> clockwise
    if (digitalRead(R_ChannelB) != R_CurrentState) {
      R_Counter --;
    }
    else {
      R_Counter ++;
    }
  }
  /*Serial.print("R_Pose: ");
    Serial.print(R_Counter);
  */
  R_LastState = R_CurrentState;

  // Change in state --> new pulse
  if (L_CurrentState != L_LastState) {
    // ChannelB different from current state --> clockwise
    if (digitalRead(L_ChannelB) != L_CurrentState) {
      L_Counter --;
    }
    else {
      L_Counter ++;
    }
  }
  /*Serial.print("  L_Pose: ");
    Serial.print(L_Counter);
  */
  L_LastState = L_CurrentState;

  // Calculate time interval
  time_current = millis();
  time_interval = time_current - time_prev;
  time_prev = time_current;

  int R_tick_change = R_Counter - R_ticks_prev;
  float R_distance = (float)R_tick_change / ticks_per_revolution;
  double R_velocity = R_distance / time_interval * 10000.0;

  int L_tick_change = L_Counter - L_ticks_prev;
  float L_distance = (float)L_tick_change / ticks_per_revolution;
  double L_velocity = L_distance /  time_interval * 10000.0;

  // Update previous tick counts for the next iteration
  R_ticks_prev = R_Counter;
  L_ticks_prev = L_Counter;
  
    Serial.print("  R_Vel: ");
    Serial.print(R_velocity);
    Serial.print("  L_Vel: ");
    Serial.print(L_velocity);
  
  delay(80);


  Output = PID_control(Kp, Ki, Kd, R_velocity, L_velocity);

  forward();
}


/**********************PID**********************/
Output_PID_t PID_control(float Kp, float Ki, float Kd, double R_velocity, double L_velocity)
{
  R_Error = SetPoint - R_velocity;
  Output.R_PID = Kp * R_Error + Kd * (R_Error - R_PreviousError);
  R_PreviousError = R_Error;

  if (Output.R_PID > 100) {
    Output.R_PID = 100;
  }
  else if (Output.R_PID < -100) {
    Output.R_PID = -100;
  }

  Serial.print((String)"      R_Error: " + R_Error + (String)"  R_PreviousError: " + R_PreviousError);

    L_Error = SetPoint - L_velocity;
  Output.L_PID = Kp * L_Error + Kd * (L_Error - L_PreviousError);
  L_PreviousError = L_Error;

  if (Output.L_PID > 100) {
    Output.L_PID = 100;
  }
  else if (Output.L_PID < -100) {
    Output.L_PID = -100;
  }

  Serial.print((String)"  L_Error: " + L_Error + (String)"  L_PreviousError: " + L_PreviousError);
  Serial.println((String)"      R_PID: " + Output.R_PID + (String)"  L_PID: " + Output.L_PID);

  return Output;

}


/**********************MOTION**********************/
void forward()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, Output.L_PID);
  analogWrite(ENB, Output.R_PID);
}
/*
void backward()
{

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, L_PID);
  analogWrite(ENB, R_PID);
}
void left()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, L_PID);
  analogWrite(ENB, R_PID);
}
void right()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, L_PID);
  analogWrite(ENB, R_PID);

}
void SStop()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

*/