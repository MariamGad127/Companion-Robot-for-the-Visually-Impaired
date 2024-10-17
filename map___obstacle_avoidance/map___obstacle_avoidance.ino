
/********************ENCODERS********************/
//right encoder
#define ChannelA 13
#define ChannelB 4
//left encoder
#define ChannelC A0
#define ChannelD A5

typedef struct {
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

/********************MAP********************/
// TODO: assign the actual value of the room and ticks per cell, or use dynamic memory allocation (preferred)
// Assume the room is 10 by 10 cells, where each cell is represented by 5 encoder counts
#define roomLength 5
#define roomWidth 5
#define ticksPerCell 5
int Map[roomLength][roomWidth];
int currentRow, currentCol;

#define Not_detected 2
#define Free 1
#define Obstacle 0

int reached_wall = 0;
int turns_counter = 0;


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

#define Speed 70
#define Offset 17

typedef enum {
  forward,
  right,
  left,
  back_then_right,
  right_forward_right,
  right_two_forward_right,

} Directions_t;

Directions_t Direction_obstacle;
Directions_t Direction_map;


/********************FUNCTION PROTOTYPES********************/
Encoders_t readEncoders();
Directions_t scanCurrentCell();
Directions_t obstacleAvoider();
void displayMap();
void forward_FB();
void backward_FB();
void right_FB();
void left_FB();
void SStop();




void setup() {
  Serial.begin(9600);

  // Rotary encoder
  pinMode(ChannelA, INPUT);
  pinMode(ChannelB, INPUT);
  pinMode(ChannelC, INPUT);
  pinMode(ChannelD, INPUT);

  // Encoder initial state
  Encoders.LastState_R = digitalRead(ChannelA);
  Encoders.LastState_L = digitalRead(ChannelC);

  Encoders = readEncoders();
  Encoders.currentTick_R = Encoders.counter_R;
  Encoders.currentTick_L = Encoders.counter_L;

  // Initialize matrix with 2 (2 --> not detected yet)
  for (int i = 0; i < roomLength; i++) {
    for (int j = 0; j < roomWidth; j++) {
      Map[i][j] = Not_detected;
    }
  }

  // First cell initialization
  currentRow = 0;
  currentCol = 1;

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

  Direction_obstacle = obstacleAvoider();
  




  switch (Direction_obstacle) {
    case forward:
      forward_FB();
      break;

    case back_then_right:
      backward_FB();
      right_FB();
      break;

    case right:
      right_FB();
      break;

    case left:
      left_FB();
      break;

    default:
      SStop();
      break;
  }
}




/************************MAP************************/
void displayMap() {
  // Display the matrix as cells
  for (int row = 0; row < roomLength; row++) {
    for (int col = 0; col < roomWidth; col++) {
      if (Map[row][col] == Obstacle) {
        Serial.print(" x ");
      } else if (Map[row][col] == Free) {
        Serial.print(" o ");
      } else {
        Serial.print(" | ");
      }
    }
    Serial.println();
  }
}


/************************READINGS************************/
Distances_t readDistances() {
  Distances_t distance;
  distance.Left = uLeft.read();
  distance.Right = uRight.read();
  distance.Front = uFront.read();
  distance.Top = uTop.read();

  return distance;
}

Encoders_t readEncoders() {

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
      Encoders.counter_L--;
    } else {
      Encoders.counter_L++;
    }
    Serial.print("    Position_L: ");
    Serial.println(Encoders.counter_L);
  }
  Encoders.LastState_L = Encoders.CurrentState_L;

  return Encoders;
}


/************************SCAN AND SEND DIRECTION************************/
Directions_t scanCurrentCell() {

  if (!reached_wall) {
    Direction_map = forward;


    // Re-read the distances from the ultrasonic sensors every 5 encoder increments
    // Odd columns
    if ((Encoders.currentTick_R % ticksPerCell == 0) && (Encoders.currentTick_R > Encoders.previousTick_R) && (currentCol % 2 != 0) && (currentRow < roomLength - 1)) {
      Serial.println("odd column");
      Distances_t Distance = readDistances();

      // Do not activate this section till the top sensor is fixed properly
      // Check for top obstacles
      if (Distance.Top < topThreshold) {
        Map[currentRow][currentCol] = Obstacle;
      } else {
        Map[currentRow][currentCol] = Free;
      }


      // Columns: 1,5,9,...
      if ((currentCol - 1) % 4 == 0) {
        Serial.println("Columns: 1,5,9,...");
        // Check for planar obstacles
        if (Distance.Left < threshold) {
          Map[currentRow][currentCol + 1] = Obstacle;
        } else {
          Map[currentRow][currentCol + 1] = Free;
        }

        if (Distance.Right < threshold) {
          Map[currentRow][currentCol - 1] = Obstacle;
        } else {
          Map[currentRow][currentCol - 1] = Free;
        }

        if (Distance.Front < threshold) {
          Map[currentRow + 1][currentCol] = Obstacle;
        } else {
          Map[currentRow + 1][currentCol] = Free;
        }
      }

      // Columns: 3,7,11,...
      else {
        Serial.println("Columns: 3,7,11,...");
        // Check for planar obstacles
        if (Distance.Left < threshold) {
          Map[(roomLength - 1) - currentRow][currentCol - 1] = Obstacle;
        } else {
          Map[(roomLength - 1) - currentRow][currentCol - 1] = Free;
        }

        if (Distance.Right < threshold) {
          Map[(roomLength - 1) - currentRow][currentCol + 1] = Obstacle;
        } else {
          Map[(roomLength - 1) - currentRow][currentCol + 1] = Free;
        }

        if (Distance.Front < threshold) {
          Map[(roomLength - 1) - currentRow + 1][currentCol] = Obstacle;
        } else {
          Map[(roomLength - 1) - currentRow + 1][currentCol] = Free;
        }
      }
    }
  }

  else {
    Direction_map = right_two_forward_right;
  }

  return Direction_map;
}



/************************OBSTACLE AVOIDER************************/
Directions_t obstacleAvoider() {
  // Read distance readings from all sides
  LeftDistance = uLeft.read();
  RightDistance = uRight.read();
  FrontDistance = uFront.read();
  TopDistance = uTop.read();

  Serial.println((String) "Front: " + FrontDistance + (String) "  Left: " + LeftDistance + (String) "  Right: " + RightDistance + (String) "  Top: " + TopDistance);

  // Top obstacles, move backward then right
  if (TopDistance < 60) {
    return back_then_right;
  }

  // Obstacle in the front, turn right
  else if (FrontDistance < threshold && RightDistance > threshold) {
    return right;
  }
  // Front and right cells are occupied, turn left
  else if (FrontDistance < threshold && RightDistance < threshold && LeftDistance > threshold) {
    return left;
  } else {
    return forward;
  }
}



/************************CONTROL WITH FEEDBACK************************/
void forward_FB() {
  Encoders = readEncoders();
  Encoders.previousTick_R = Encoders.counter_R;
  Encoders.previousTick_L = Encoders.counter_L;
  // Serial.println((String)"  previousTick_R: " + Encoders.previousTick_R + (String)"  previousTick_L: " + Encoders.previousTick_L);

  while ((Encoders.currentTick_L - Encoders.previousTick_L < 5) || (Encoders.currentTick_R - Encoders.previousTick_R < 5)) {
    if (Encoders.currentTick_L - Encoders.previousTick_L < 5) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, Speed);
    }
    if (Encoders.currentTick_R - Encoders.previousTick_R < 5) {
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

void backward_FB() {
  Encoders = readEncoders();
  Encoders.previousTick_R = Encoders.counter_R;
  Encoders.previousTick_L = Encoders.counter_L;
  //Serial.println((String)"  previousTick_R: " + Encoders.previousTick_R + (String)"  previousTick_L: " + Encoders.previousTick_L);

  while ((Encoders.previousTick_L - Encoders.currentTick_L < 5) || (Encoders.previousTick_R - Encoders.currentTick_R < 5)) {
    if (Encoders.previousTick_L - Encoders.currentTick_L < 5) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, Speed);
    }
    if (Encoders.previousTick_R - Encoders.currentTick_R < 5) {
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

  while ((Encoders.currentTick_L - Encoders.previousTick_L < 10) || (Encoders.currentTick_R - Encoders.previousTick_R > -10)) {
    if (Encoders.currentTick_L - Encoders.previousTick_L < 10) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, Speed);
    }
    if (Encoders.currentTick_R - Encoders.previousTick_R > -10) {
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

  while ((Encoders.currentTick_L - Encoders.previousTick_L > -10) || (Encoders.currentTick_R - Encoders.previousTick_R < 10)) {
    if (Encoders.currentTick_L - Encoders.previousTick_L > -10) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, Speed);
    }
    if (Encoders.currentTick_R - Encoders.previousTick_R < 10) {
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
