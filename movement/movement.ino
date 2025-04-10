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
float stop_dist = 5;    //stopping distance
float center_range[2] = {11.0,16.0}; //distance from wall
unsigned long duration_maze = 1400;  //duration to next cell
unsigned long duration_enter = 1250; //duration to enter maze
bool left_wall = true;      //is there a wall to left (assume to be true at first)
bool front_wall = false;      //is there a wall in front

// Maze stuff
int position[2] = {4,1};      //position in maze
int direction = 1;            //direction it is facing; 1-N, 2-W, 3-S, 4/0-E. Mod 4 to get direction
int recursion = 2;            //how many times to repeat recursion (testing only)
bool maze_finished = false;        //has the end of the maze been reached
bool moved = false;

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
  look_left();

  // Set ultrasonic pins
  pinMode(echo, INPUT);
  pinMode(trigger, OUTPUT);

  // Start serial communication
  Serial.begin(9600);
}

void loop() {

  /* bool end_line=false;          //end of line tracking condition
  while (!end_line) {
    end_line = photosensor(line_speed);   //start line tracking until condition is met
  }
  Serial.println("LINE DONE");
  line = false;
  delay(3000);

  transition();
  Serial.println("MAZE START"); */

  follow_left();
  
  /* look_fw();
  while (true) {
    sense_dist();
    delay(1000);
  } */

  //orientation();

  //rotateL90();
  //next_cell(duration_maze);

  /* while(true) {
    check_val();
    Serial.println();
    delay(500); 
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
      for (int i=0; i<500; i++) {      //2000 gives a wide enough search radius. Can be reduced
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
        for (int i=0; i<1000; i++) {
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
  center = center_value >= 600;
  right = right_value >= 600;
  left = left_value >= 600;

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
    delay(60);
    stop(5);
  }
}

//SERVO MOVEMENT
bool look_fw() {
  servo.write(55);
  delay(250);     //wait for servo to finish turning
  return true;
}

void look_left() {
  servo.write(168);
  delay(250);     //wait for servo to finish turning
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
    if (ld>center_range[1]+2) {                         //no wall to left in maze
      left_wall = false;                                //dont try to correct
      break;
    } else {
      left_wall = true;
      startPause = millis();
      if (ld<center_range[0]) {                           //too close to wall
        backward(maze_speed);                                       //move a bit back
        delay(75);
        turn_right(maze_speed, maze_speed);                                   //turn out
        delay(50);                                        //turn angle to fix alignment, may change
        stop(5);            //adjustment factor moved
      } else if (ld>center_range[1]) {
        backward(maze_speed);
        delay(75);
        turn_left(maze_speed, maze_speed);
        delay(50);
        stop(5);
      }
      endPause = millis();
      duration += endPause-startPause+45;                 //extend duration by time taken to readjust and a correction factor
      Serial.print("Test range ");
      ld = sense_dist();                                  //test if still outside range
    }
  }
  forward(maze_speed);
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
  while (!maze_finished) {
    moved = false;
    //Check left wall first which has already been done while moving to next cell
    look_left();
    if (sense_dist() > 19) {
      Serial.println("No left");
      rotateL90();              //turn left if there is no left wall
      direction += 1;           //updating direction
      next_cell(duration_maze);    //go to next cell
      check_end();
    }
    //Check front wall
    if (!moved) {
      look_fw();
      if (sense_dist() > stop_dist+20) {
        Serial.println("No front wall");
        front_wall = false;             //no front wall within stop distance + marign for error
        next_cell(duration_maze);
        check_end();
      }
      //Readjust
      else if (sense_dist() < stop_dist+15) {
        front_wall = true;                //there is a wall in front
        while (sense_dist() <= stop_dist-2) {    //recenter if too close to wall. -2 for lientiency and so it isnt always readjusting
          backward(maze_speed);
          delay(50);
          stop(5);
        }
        while (sense_dist() > stop_dist) {       //a little back behind the stop distance
          forward(maze_speed);
          delay(50);
          stop(5);
        }
      }
    }
    //Check right wall
    if (!moved) {
      rotateR90(); 
      if (sense_dist() > stop_dist+20) {
        Serial.println("No right wall");
        front_wall = false;             //no right wall (relative to starting position)
        next_cell(duration_maze);
        check_end();
      }
      //Blocked on all 3 sides
      else {
        Serial.println("Boxed in");
        rotateR90();
        next_cell(duration_maze);
        check_end();
      }
    }
  }
}

// Move into the next cell while staying relatively straight 
void next_cell(unsigned long duration) {
  Serial.println("next cell");
  forward(maze_speed);        //move forward
  unsigned long startTime = millis();
  while (millis()-startTime < duration) {
    duration = orientation(duration);       //keep itself straight
  }
  stop(5);
  moved = true;
}

void rotateL90() {
  turn_left(maze_speed, maze_speed);
  delay(1050);      //timing to reach 90. 1010, timing might change with power in battery?
  stop(5);
}

bool rotateR90() {
  turn_right(maze_speed, maze_speed);
  delay(1050);      //timing to reach 90
  stop(5);
  return true;
}
// Moving between line tracking and maze

/* Sudo Code

  photosensor until can not longer detect line (entered maze)
  enter cell with different duration to center (find that time);

*/
void transition() {
  bool entered = false;
  forward(line_speed);
  delay(500);
  stop(5);
  while (!entered) {
    entered = photosensor(line_speed);
  }
  next_cell(duration_enter);
}

void check_end() {
  check_val();                      //check if it has reach the end ramp
  Serial.println();
  if (!left && !center && !right) { //Use !left && !center && !right. Currently for testing
    end_maze();                     //it has reached the end
  } else {
    return;
  }
}

void end_maze() {
  forward(line_speed);
  Serial.println("MAZE COMPLETE");
  maze_finished = true;
  delay(2000);
  stop(5);
}
