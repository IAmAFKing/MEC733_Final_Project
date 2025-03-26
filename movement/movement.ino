// Motor driver pins
#define ENA 5  // Left motor speed control
#define ENB 6  // Right motor speed control
#define IN1 7  // Left motor direction
#define IN2 8
#define IN3 9  // Right motor direction
#define IN4 11
#define STBY 3 // Standby pin for enabling motor driver

// Speed of the motors (0-255)
int motor_speed = 255;

void setup() {
  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable motor driver
  digitalWrite(STBY, HIGH);
}

void loop() {
  // Left wheels move forward
  analogWrite(ENA, motor_speed);
  digitalWrite(IN1, HIGH);   // HIGH = FORWARD, LOW = BACKWARD
  digitalWrite(IN2, HIGH);   // HIGH = FORWARD, LOW = BACKWARD

  // Right wheels move forward
  analogWrite(ENB, motor_speed);
  digitalWrite(IN3, HIGH); // HIGH = FORWARD, LOW = BACKWARD
  digitalWrite(IN4, HIGH); // HIGH = FORWARD, LOW = BACKWARD
}