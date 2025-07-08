# Final Project for MEC 733
In collaboration with Jackson Potter and Ivan Lau
## Objective
Program a robot car to follow a path and solve a maze. The car is the ELEGOO UNO R3 Project Smart Robot Car Kit V4 with UNO R3.

The car must first follow a black line from Point A to B. Then the car will enter a maze and must complete the maze without collision and under 45 seconds
## Details
1. The maze is a 4x4 with enterance and exit on the outsides
2. The car is equiped with 4 individually powered fixed wheels, ultrasonic sensor on a servo motor, 3 photosensors, and a gyroscope and accelerometer
## Process
There are 2 parts of this project to take on: line tracking and maze solving.
### Line tracking
Line tracking requires the car to follow a black line of tape as it changes directions.

The theory behind it is very simple: using the photosensors, find where the line is relative to the center of the car and move towards it accordingly. The problem comes when the car doesn't go perfectly straight or it veers off course due to sharper turns. Doing multiple incriments of turns and movements helps keep the car accurate and consistent but finding the right timing and angle of the turns requires a bit of testing. If it ever does veer off course and completely loses the black line, a search mode will help it to get back on track.
### Maze solving
The maze is the hardest part and worth the most. First there is the transition into the maze, then there is actually solving it.

The transition between the line tracking and the maze was relatively easy. There was a blackline going up the ramp so using the photosensor would help get it into the maze easily. The maze floor was made of translucent acrylic so the stopping values would be very similar to the black line. All this gets the car into the maze at the same spot most of the time.

The most annoying part is not actually solving the maze but doing it consistently. The thing with this car is that as the battery power goes down, so does the motor power meaning the car will travel less distance and turn less each time you run it. This was infuriating as the numbers would constantly be changing and getting it right would be near impossible.

Aside from that, there are many other factors that would mess up the maze solving. Specifically not driving straight. The wheels on the car are not screwed in perfectly parallel and straight so the car will drift off the straight path. To combat this, the ultrasonic sensor is used to track a wall on the side. If it strays too far from the wall, it will move back a bit and turn in to attempt to get it back straight. If it is too close to the wall, it will move back and turn out. It will also attempt to center itself in the cell by checking the front wall and moving back or forward to get center.

The rest of the maze solving is just using a follow left algorithm to autonomously solve the maze and combining it with the wall tracking to keep it center and straight.
## Results
The line tracking portion was done flawlessly and we were able to complete the maze in under 40 seconds only hitting the wall 2 times. However, it was not autonomous and had to use some scuff to complete the maze at all and under the time limit. First, due to the problems of inconsistency mentioned above, we could not get it to complete the maze autonomously consistently and without constantly hitting the wall. The second problem was that the follow left algorithm we chose would be the longest path that we could have taken meaning it wouldn't finish in the time requirement.

Our solution was to have the car overshoot the cell so that it will skip past the dead ends and use the front adjustment to pull to towards the next cell and line it up perfectly to keep going. This was only possible due to the maze design. There are 2 videos, one for the proper autonomous run that we were only able to get 2 times, and our demo run which we cheesed.
## Improvements
Many of the improvements would be in the maze solving portion. First, the inconstiency problem needs to be solved with a better battery. Aside from hardware, one improvement that could be added is a better solving alg. Obviously the follow left works but it could end up taking the longest path like in our case or it could miss the exit because it wants to turn left first. Or if the maze was designed where the finish is in the center of the maze, that would stop the car from ever reaching the finish.

An improvement would be to track the location of the car everytime it moves into a new cell. A small improvement would be to check if it is ever in the exit cell and if it is, just turn and leave. This can be expanded on by using a whole new algorithm where it will always attempt the shortest path and update the shortest path as walls get put in its way.
## Conclusion
This was a really fun project to work on and we learned a lot about problem solving and control systems to complete the tasks. Many improvements and different approaches could be used and I hope to come back and attempt this challenge again.