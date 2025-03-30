// Motor driver pins
#define ENA 5  // Left motor speed control
#define ENB 6  // Right motor speed control
#define IN1 7  // Right motors
#define IN2 8  // Left motors

#define STBY 3 // Standby pin for enabling motor driver

// Line sensor pins
#define RIGHT_SENSOR A0  
#define CENTER_SENSOR A1
#define LEFT_SENSOR A2

// Default speed of the motors (0-255)
int motor_speed = 70;

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
  bool end_line = false;

  while (!end_line) {
    end_line = photosensor();
  }

  while (true) {
  }
}

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

void turn_left() {
  analogWrite(ENA, motor_speed - 50);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}

void turn_right() {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed - 50);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}

bool photosensor() {
  int right_value = analogRead(RIGHT_SENSOR);
  int left_value = analogRead(LEFT_SENSOR);
  int center_value = analogRead(CENTER_SENSOR);

  bool right_side = false;
  bool left_side = false;
  bool center = false;

  Serial.print("Left sensor: ");
  Serial.print(left_value);
  Serial.print("\t center sensor: ");
  Serial.print(center_value);
  Serial.print("\t right sensor: ");
  Serial.println(right_value);
  delay(1000);

  // Line-following logic
  if (right_value >= 800 && right_value <= 1000) {  
    right_side = true;
  }
  if (left_value >= 800 && left_value <= 1000) {
    left_side = true;
  }
  if (center_value >= 800 && center_value <= 1000) {
    center = true;
  }

  if (right_side && left_side) {
    stop();
    return true;
  } else if (right_side) {
    turn_right();
  } else if (left_side) {
    turn_left();
  } else if (center) {
    forward();
  } else {
    stop();
  }
  return false;
}