#include <Servo.h>
#include <avr/io.h>
// Motor driver pins
#define ENA 5  // Right motor speed control
#define ENB 6  // Left motor speed control
#define IN1 7  // Right motor
#define IN2 8  // Left motor
#define STBY 3 // Standby pin for enabling motor driver

// Speed of the motors (0-255)
int motor_speed = 63;

// Photosensor pins
#define RIGHT_SENSOR A0  
#define CENTER_SENSOR A1
#define LEFT_SENSOR A2

// Photosensor values
int right_value;
int left_value;
int center_value;

bool center;
bool right;
bool left;

// Servo pins
#define SRV 10

// Servo class name
Servo servo;

// Ultrasonic pins
#define echo 12         //recieve pulse
#define trigger 13      //send pulse

// Ultrasonic values
float rd = 0;           //right distance
float ld = 0;           //left distance
float fd = 0;           //forward distance
float stop_dist = 2;    //stopping distance
float center_range[2] = {11.0,17.0}; //distance from wall
unsigned long duration = 1450;  //duration to next cell

void setup() {
  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable motor driver
  digitalWrite(STBY, HIGH);

  // Set sensor as input
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(CENTER_SENSOR, INPUT);

  // Set servo as output
  servo.attach(SRV);
  pinMode(SRV, OUTPUT);

  // Set ultrasonic pins
  pinMode(echo, INPUT);
  pinMode(trigger, OUTPUT);

  // Start serial communication
  Serial.begin(9600);
}

void loop() {

  /* bool end_line=false;          //end of line tracking condition
  while (!end_line) {
    end_line = photosensor();   //start line tracking until condition is met
  }
  Serial.println("DONE");
  delay(3000); */
  
  /* look_left();
  while (true) {
    sense_dist();
    delay(1000);
  } */

  //orientation();

  //rotateL90();

  next_cell();

  // Loop prevention
  while (true) {
  }
}

//MOVEMENT FUNCTIONS
void forward() {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}

void backward() {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
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

//LINE TRACKING 
bool photosensor() {
  check_val();                            //checks values of photosensors

  Serial.print("Left sensor: ");
  Serial.print(left_value);
  Serial.print(" \t center sensor: ");
  Serial.print(center_value);
  Serial.print("\t right sensor: ");
  Serial.print(right_value);

  // Line-following logic
  // Start line tracking if end condition is not met (reaches horizontal black line)
  if (!(left && right)) {
    // When black line is near the center
    if (center) {
      check_error();                    //checks for errors in how center it is
    }
    // Black line completely on left side
    else if (left) {
      Serial.println(" left rotate");
      turn_left(63);                    //rotate left to compensate
    }
    // Black line completely on right side
    else if (right) {
      Serial.println(" right rotate");
      turn_right(63);                   //rotate right to compensate
    }
    // Black line nowhere to be found
    else {                              //enter search mode
      Serial.println(" search");
      // Check for black line at each turn interval
      for (int i=0; i<1000; i++) {      //2000 gives a wide enough search radius. Can be reduced
        // Trurn a bit and check sensor values
        turn_left(63);
        check_val();
        if (center) {
          // Break out of search loop and resume line tracking
          stop(10);
          break;
        }
      }
      // Only check right if nothing was found on the left side
      // Repeat above steps
      if (!center) {
        for (int i=0; i<2000; i++) {
          turn_right(63);
          check_val();
          if (center) {
            stop(10);
            break;
          }
        }
      }
      // Stop if cannot find line on either side
      if (!center) {
        stop(1000);
        Serial.println("lost");
        while(true){
        }
      }
    }
  }
  // End condition where it reaches the horizontal black line
  // May need to be changed as starting position might have a black line too
  else {
    stop(100);
    Serial.println();
    return true;        //end conition has been met
  }
  return false;         //end condition has not been met

  delay(10);
}

// Check values from photosensor
void check_val() {
  right_value = analogRead(RIGHT_SENSOR);
  left_value = analogRead(LEFT_SENSOR);
  center_value = analogRead(CENTER_SENSOR);
  
  // Black line range between 700 and 950. Record if the sensors detect something in that range
  center = center_value >= 700 && center_value <= 950;
  right = right_value >= 700 && right_value <= 950;
  left = left_value >= 700 && left_value <= 950;
}

// Checking left or right errors when center detects the line
void check_error() {
  if (left)
  {
    Serial.println(" left turn");
    turn_left(0);                   //slower turn left to compensate
    delay(120);
    stop(5);
  }
  else if (right)
  {
    Serial.println(" right turn");
    turn_right(0);                  //slower turn right to compensate
    delay(120);
    stop(5);
  }
  else
  {
    Serial.println(" forward");
    forward();                      //no error go straight ahead
  }
}

//SERVO MOVEMENT
void look_fw() {
  servo.write(55);
}

void look_left() {
  servo.write(168);
}

//ULTRASONIC SENSOR
float sense_dist() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delay(10);
  digitalWrite(trigger, LOW);
  float dist = (pulseIn(echo,HIGH))/58.3;
  Serial.print("Dist: ");
  Serial.print(dist, 3);
  Serial.println("cm");
  return dist;
}

//MAZE ALGORITHM

/* SUDO CODE

enters cell
  needs to enter going straight
  ik the line tracking is really wild with the turns, gonna need to change that
as it enters, it checks dist to left wall
  adjusts to get closer to wanted distance
    greater than expected, turn left, measure value, compare to previous value measured
      keep turning until going down
      go straight
    less than expected, turn right, same as above

Can test functionality with line tracker
  works

*/

// Keep center/straight
void orientation() {
  look_left();
  unsigned long startPause;
  unsigned long endPause;
  int counter=0;
  ld = sense_dist();                                    //measure current distance
  while (ld<center_range[0] || ld>center_range[1]) {    //outside range
    if (ld<center_range[0]) {                           //too close to wall
      startPause = millis();
      backward();                                       //move a bit back
      delay(50);
      turn_right(63);                                   //turn out
      delay(50);                                        //turn angle to fix alignment, may change
      stop(5);
      endPause = millis();
      duration += (endPause-startPause);
      counter++;
    } else if (ld>center_range[1]) {
      backward();
      delay(50);
      turn_left(63);
      delay(50);
      stop(5);
    }
    Serial.print("Test range ");
    ld = sense_dist();                                  //test if still outside range
  }
  duration += counter*55;                               //correction factor
  forward();
  Serial.print("End ");
}

/* SUDO CODE

car moves forward a certain distance to get to cell
  meanwhile it is also checking to make sure it is straight/in line
    cant use delay, need to use internal timer that runs seperate
once it reaches the "supposed" center of the next cell
  check forward
    if wall, readjust to correct distance
      check for left wall
      if left wall rotate 90 right
        check forward
        if forward, rotate 90 right and go to next cell
      if no left wall, rotate 90 left, go next cell
    if no front wall, check left
      if left wall, go to next cell
      if no left wall, rotate 90 left and go next cell


*/

void follow_left() {

}

void next_cell() {
  forward();        //move forward
  unsigned long startTime = millis();
  while (millis()-startTime < duration) {
    orientation();
  }
  stop(5);
}

void rotateL90() {
  turn_left(63);
  delay(1010);      //timing to reach 90
  stop(5);
}

void rotateR90() {
  turn_right(63);
  delay(1010);      //timing to reach 90
  stop(5);
}