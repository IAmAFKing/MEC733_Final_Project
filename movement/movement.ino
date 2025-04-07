#include <Servo.h>
#include <avr/io.h>
// Motor driver pins
#define ENA 5  // Right motor speed control
#define ENB 6  // Left motor speed control
#define IN1 7  // Right motor
#define IN2 8  // Left motor
#define STBY 3 // Standby pin for enabling motor driver

// Speed of the motors (0-255)
int line_speed = 103;
int maze_speed = 63;

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

bool line=true;

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
unsigned long duration_maze = 1450;  //duration to next cell
unsigned long duration_enter = 1225; //duration to enter maze
bool left_wall = false;      //is there a wall to left
bool front_wall = true;      //is there a wall in front

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

  bool end_line=false;          //end of line tracking condition
  while (!end_line) {
    end_line = photosensor(line_speed);   //start line tracking until condition is met
  }
  Serial.println("DONE");
  line = false;
  delay(3000);

  transition();
  
  /* look_left();
  while (true) {
    sense_dist();
    delay(1000);
  } */

  //orientation();

  //rotateL90();

  //next_cell(duration_enter);

  /* while(true) {
    check_val();
  } */

  // Loop prevention
  while (true) {
  }
}

//MOVEMENT FUNCTIONS
void forward(int motor_speed) {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}

void backward(int motor_speed) {
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

void turn_left(int motor_speed, int speed2) {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, speed2);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void turn_right(int motor_speed, int speed2) {
  analogWrite(ENA, speed2);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

//LINE TRACKING 
bool photosensor(int speed) {
  check_val();                            //checks values of photosensors

  // Line-following logic
  // Start line tracking if end condition is not met (reaches horizontal black line)
  if (!(left && right)) {
    // When black line is near the center
    if (center) {
      check_error(speed);                    //checks for errors in how center it is
    }
    // Black line completely on left side
    else if (left) {
      Serial.println(" left rotate");
      turn_left(speed, speed);                    //rotate left to compensate
      delay(35);
      stop(5);
    }
    // Black line completely on right side
    else if (right) {
      Serial.println(" right rotate");
      turn_right(speed, speed);                   //rotate right to compensate
      delay(35);
      stop(5);
    }
    // Black line nowhere to be found
    else {                              //enter search mode
      Serial.println(" search");
      // Check for black line at each turn interval
      for (int i=0; i<1000; i++) {      //2000 gives a wide enough search radius. Can be reduced
        // Trurn a bit and check sensor values
        turn_left(speed, speed);
        delay(35);
        stop(5);
        check_val();
        if (center) {
          // Break out of search loop and resume line tracking
          stop(5);
          break;
        }
      }
      // Only check right if nothing was found on the left side
      // Repeat above steps
      if (!center) {
        for (int i=0; i<2000; i++) {
          turn_right(speed, speed);
          delay(35);
          stop(5);
          check_val();
          if (center) {
            stop(5);
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
    stop(5);
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
  center = center_value >= 675 && center_value <= 950;
  right = right_value >= 675 && right_value <= 950;
  left = left_value >= 675 && left_value <= 950;

  Serial.print("Left sensor: ");
  Serial.print(left_value);
  Serial.print(" \t center sensor: ");
  Serial.print(center_value);
  Serial.print("\t right sensor: ");
  Serial.print(right_value);
}

// Checking left or right errors when center detects the line
void check_error(int speed) {
  if (left)
  {
    Serial.println(" left turn");
    turn_left(speed, 0);                   //slower turn left to compensate
    delay(25);
    stop(5);
  }
  else if (right)
  {
    Serial.println(" right turn");
    turn_right(speed, 0);                  //slower turn right to compensate
    delay(25);
    stop(5);
  }
  else
  {
    Serial.println(" forward");
    forward(speed);                      //no error go straight ahead
    delay(65);
    stop(5);
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
unsigned long orientation(unsigned long duration) {
  look_left();
  unsigned long startPause;
  unsigned long endPause;
  ld = sense_dist();                                    //measure current distance
  
  while (ld<center_range[0] || ld>center_range[1]) {    //outside range
    startPause = millis();
    if (ld<center_range[0]) {                           //too close to wall
      backward(maze_speed);                                       //move a bit back
      delay(50);
      turn_right(maze_speed, maze_speed);                                   //turn out
      delay(50);                                        //turn angle to fix alignment, may change
      stop(5);            //adjustment factor moved
    } else if (ld>center_range[1]) {
      backward(maze_speed);
      delay(50);
      turn_left(maze_speed, maze_speed);
      delay(50);
      stop(5);
    }
    endPause = millis();
    duration += endPause-startPause+37;                 //extend duration by time taken to readjust and a correction factor
    Serial.print("Test range ");
    ld = sense_dist();                                  //test if still outside range
  }
  forward(maze_speed);
  Serial.print("End ");
  return duration;
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

// Move into the next cell while staying relatively straight 
void next_cell(unsigned long duration) {
  forward(maze_speed);        //move forward
  unsigned long startTime = millis();
  while (millis()-startTime < duration) {
    duration = orientation(duration);
  }
  stop(5);
}

void rotateL90() {
  turn_left(maze_speed, maze_speed);
  delay(1010);      //timing to reach 90
  stop(5);
}

void rotateR90() {
  turn_right(maze_speed, maze_speed);
  delay(1010);      //timing to reach 90
  stop(5);
}
// Moving between line tracking and maze

/* Sudo Code

  photosensor until can not longer detect line (entered maze)
  enter cell with different duration to center (find that time);

*/
void transition() {
  bool entered = false;
  forward(line_speed);
  delay(100);
  stop(5);
  while (!entered) {
    entered = photosensor(maze_speed);
  }
  next_cell(duration_enter);
}
