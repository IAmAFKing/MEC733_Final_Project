// Motor driver pins
#define ENA 5  // Right motor speed control
#define ENB 6  // Left motor speed control
#define IN1 7  // Right motor
#define IN2 8  // Left motor

#define STBY 3 // Standby pin for enabling motor driver

// Line sensor pins
#define RIGHT_SENSOR A0  
#define CENTER_SENSOR A1
#define LEFT_SENSOR A2

// Default speed of the motors (0-255)
int motor_speed = 63;

void setup() {
  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Set sensor as input
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(CENTER_SENSOR, INPUT);

  // Enable motor driver
  digitalWrite(STBY, HIGH);

  // Start serial communication
  Serial.begin(9600);
}

void loop() {
  bool end_line = false;        //condition when the line following segment has been complete

  while (!end_line) {
    end_line = photosensor();
  }

  /* Serial.print("done");
  Serial.flush(); */

  //Stop loop
  while (true) {
  }
}

/* Movement functions */
void forward() {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}

void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void rotate_left() {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void rotate_right() {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void turn_left(int motor2) {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor2);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void turn_right(int motor2) {
  analogWrite(ENA, motor2);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

/* Line tracking algorithm.
  Maybe could have split photosensor and line tracking into 2 different functions */
bool photosensor() {
  //Read values from photo sensor
  int right_value = analogRead(RIGHT_SENSOR);
  int left_value = analogRead(LEFT_SENSOR);
  int center_value = analogRead(CENTER_SENSOR);

  //Set boolean if sensors are sensing the black line within a range
  bool center = center_value >= 700 && center_value <= 950;
  bool right = right_value >= 600 && right_value <= 950;
  bool left = left_value >= 600 && left_value <= 950;

  Serial.print("Left sensor: ");
  Serial.print(left_value);
  Serial.print("\t center sensor: ");
  Serial.print(center_value);
  Serial.print("\t right sensor: ");
  Serial.println(right_value);

  // Line-following logic
  // Check to see if the end has been reached
  /* if (left && center && right) {
    stop();
    return true;
  } */
  if (center) {
    //Start small adjustment if there is a little left or right error but the center still reads fine
    if (right) {
      turn_right(31);
    } else if (left) {
      turn_left(31);
    } else {  //go forward if no errors
      forward();
    }
  }
  //Make bigger adjustments if center is not on the line
  else if (right) {
    turn_right(63);
  } else if (left) {
    turn_left(63);
  }
  //None of the sensors see the black
  else if (!center && !right && !left) { //search mode
      //Rotate left a bit and continuously check for the line
      for (long i=0; i<20000; i++) {
        rotate_left();
        if (left_value >= 600 && left_value <= 950) {
          photosensor();
        }
      }
      //Rotate to the right side to check
      for (long i=0; i<20000; i++) {
        rotate_right();
        if (right_value >= 600 && right_value <= 950) {
          photosensor();
        }
      }
      /* //Rotate left a bit and continuously check for the line
      for (long i=0; i<20000; i++) {
        rotate_left();
        if (left) {
          photosensor();
        }
      } */
    
  }
  delay(10);
}