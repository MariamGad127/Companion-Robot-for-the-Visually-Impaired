/********************ENCODERS********************/
//right encoder
#define ChannelA 13
#define ChannelB 4
//left encoder
#define ChannelC A5
#define ChannelD A0

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
#define roomLength 15
#define roomWidth 7
#define ticksPerCell 10
int Map[roomLength][roomWidth];
int currentRow, currentCol;

#define Not_detected 2
#define Free 1
#define Obstacle 0

int turns_counter = 0;
int cell_is_done = 0;

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

#define Speed 80
#define Offset 15

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



/********************FUNCTION PROTOTYPES********************/
Encoders_t readEncoders();
Directions_t scanCurrentCell();
Directions_t obstacleAvoider();
void displayMap();
void printDirection();
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

  forward_FB();
}



void loop() {

  Encoders = readEncoders();
  Encoders.currentTick_R = Encoders.counter_R;
  Encoders.currentTick_L = Encoders.counter_L;

  Direction_map = scanCurrentCell();

  switch (Direction_map) {
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

    case left_two_forward_left:
      left_FB();
      left_FB();
      forward_FB();
      forward_FB();
      left_FB();
      left_FB();
      forward_FB();
      break;

    case right_two_forward_right:
      right_FB();
      right_FB();
      forward_FB();
      forward_FB();
      right_FB();
      right_FB();
      forward_FB();
      break;

    default:
      SStop();
      break;
  }

  //printDirection();

  Encoders.previousTick_R = Encoders.currentTick_R;
  Encoders.previousTick_L = Encoders.currentTick_L;
  Encoders = readEncoders();
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


Distances_t readDistances() {
  Distances_t distance;
  distance.Left = uLeft.read();
  distance.Right = uRight.read();
  distance.Front = uFront.read();
  distance.Top = uTop.read();

  return distance;
}


void printDirection() {
  switch (Direction_map) {
    case forward:
      Serial.println("  forward");
      break;
    case left:
      Serial.println("  left");
      break;
    case right:
      Serial.println("  right");
      break;
    case back_then_right:
      Serial.println("  back_then_right");
      break;
    case right_forward_left:
      Serial.println("  right_forward_left");
      break;
    case right_two_forward_right:
      Serial.println("  right_two_forward_right");
      break;
    case left_two_forward_left:
      Serial.println("  left_two_forward_left");
      break;
    default:
      break;
  }
}

void avoid_and_return() {
  Distances_t Distance2 = readDistances();
  if (Distance2.Right > threshold) {
    right_FB();
    right_FB();
    forward_FB();
    forward_FB();
    Distance2 = readDistances();

    if (Distance2.Left > threshold) {
      left_FB();
      left_FB();
      forward_FB();
      forward_FB();
      Distance2 = readDistances();

      if (Distance2.Left > threshold) {
        left_FB();
        left_FB();
        forward_FB();
        forward_FB();
        Distance2 = readDistances();

        if (Distance2.Right > threshold) {
          right_FB();
          right_FB();
          forward_FB();
          forward_FB();
        }
      }
    }
  }
}

Directions_t scanCurrentCell() {
  // TODO: replace increment on rows by forward() and on rows by left and right, and increment in the motion functions

  // Re-read the distances from the ultrasonic sensors every 5 encoder increments
  // Odd columns
  // Wall not reached
  if (cell_is_done /*(Encoders.currentTick_R % ticksPerCell == 0)*/ && (Encoders.currentTick_R > Encoders.previousTick_R) && (currentCol % 2 != 0) && (currentRow < roomLength - 1)) {
    Serial.println("odd column");
    Distances_t Distance = readDistances();

    // Columns: 1,5,9,...
    if ((currentCol - 1) % 4 == 0) {
      Serial.println("Columns: 1,5,9,...");

      // Do not activate this section till the top sensor is fixed properly
      // Check for top obstacles
      if (Distance.Top < topThreshold) {
        Map[currentRow][currentCol] = Obstacle;
      } else {
        Map[currentRow][currentCol] = Free;
      }

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

      // Do not activate this section till the top sensor is fixed properly
      // Check for top obstacles
      if (Distance.Top < topThreshold) {
        Map[(roomLength - 1) - currentRow][currentCol] = Obstacle;
      } else {
        Map[(roomLength - 1) - currentRow][currentCol] = Free;
      }

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
        Map[(roomLength - 1) - currentRow - 1][currentCol] = Obstacle;
        avoid_and_return();
      } else {
        Map[(roomLength - 1) - currentRow - 1][currentCol] = Free;
      }
    }

    displayMap();
    Serial.println();

    currentRow++;
    Direction_map = forward;

  }

  // Even columns
  else if (cell_is_done /*(Encoders.currentTick_R % ticksPerCell == 0)*/ && (Encoders.currentTick_R > Encoders.previousTick_R)) {
    Serial.println("even column");
    // TODO: trun right if currentCol is divisible by 4, otherwise trun left. Then, skip one column with forward()
    currentCol += 2;
    currentRow = 0;

    if (turns_counter % 2 == 0) {
      Direction_map = right_two_forward_right;
      turns_counter++;

    } else {
      Direction_map = left_two_forward_left;
      turns_counter++;
    }
  }

  else {
    Direction_map = stopp;
  }

  return Direction_map;
}


/************************CONTROL WITH FEEDBACK************************/
void forward_FB() {
  Encoders = readEncoders();
  Encoders.previousTick_R = Encoders.counter_R;
  Encoders.previousTick_L = Encoders.counter_L;
  // Serial.println((String)"  previousTick_R: " + Encoders.previousTick_R + (String)"  previousTick_L: " + Encoders.previousTick_L);

  //Serial.write('F');

  while ((Encoders.currentTick_L - Encoders.previousTick_L <= 10) && (Encoders.currentTick_R - Encoders.previousTick_R <= 10)) {
    if (Encoders.currentTick_L - Encoders.previousTick_L <= 10) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, Speed);
    }
    if (Encoders.currentTick_R - Encoders.previousTick_R <= 10) {
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

  //Serial.write('B');

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

  //Serial.write('R');

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



void left_FB() {
  Encoders = readEncoders();
  Encoders.previousTick_R = Encoders.counter_R;
  Encoders.previousTick_L = Encoders.counter_L;
  // Serial.println((String)"  previousTick_R: " + Encoders.previousTick_R + (String)"  previousTick_L: " + Encoders.previousTick_L);
  
  //Serial.write('L');

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
