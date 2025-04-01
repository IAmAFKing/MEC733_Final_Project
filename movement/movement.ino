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

// Speed of the motors (0-255)
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
  bool end_line=false;
  while (!end_line) {
    end_line = photosensor();
  }
  Serial.println("DONE");
  while (true) {
  }
}

void forward() {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}

void stop(int time) {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(time);
}

/* void rotate_left() {
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
} */

void turn_left(int speed2) {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, speed2);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void turn_right(int speed2) {
  analogWrite(ENA, speed2);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

bool photosensor() {
  int right_value = analogRead(RIGHT_SENSOR);
  int left_value = analogRead(LEFT_SENSOR);
  int center_value = analogRead(CENTER_SENSOR);

  bool center = center_value >= 700 && center_value <= 950;
  bool right = right_value >= 700 && right_value <= 950;
  bool left = left_value >= 700 && left_value <= 950;

  Serial.print("Left sensor: ");
  Serial.print(left_value);
  Serial.print(" \t center sensor: ");
  Serial.print(center_value);
  Serial.print("\t right sensor: ");
  Serial.print(right_value);

  // Line-following logic
  if (!(left && right)) {
    if (center) {
      if (left) {
        Serial.println(" left turn");
        turn_left(0);
      }else if (center && right) {
        Serial.println(" right turn");
        turn_right(0);
      } else {
        Serial.println(" forward");
        forward();
      }
    } else if (left) {
      Serial.println(" left rotate");
      turn_left(63);
    } else if (right) {
      Serial.println(" right rotate");
      turn_right(63);
    } else { //search mode
      Serial.println(" nothing");
      stop(100);
    }
  } else {
    stop(100);
    return true;
  }
  return false;
 /*  if (center) {  
    forward();
  } else if (right) {  
    turn_right(31);
  } else if (left) {
    turn_left(31);
  } else if (!center && !right && !left) {
    stop(1000);
  } */

  delay(10);
  //some random test

}