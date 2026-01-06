# Autonomous Pool Playing Robot
VEXIQ C++ Autonomous Pool Playing Robot

## Overview
The robot is designed to repeatedly pocket a light-coloured pool ball on a 44 by 88-inch pool table until the ball is successfully cleared. The program begins by receiving input from the controller that identifies the robot’s initial position, which is assigned by the user at one of the four corners of the pool table. The robot, built with a three-motor drivetrain, then proceeds by navigating to the center using motor encoders along with the measured dimensions of the pool table. This center is set as the origin of the coordinate system, which serves as a reference for subsequent calculations. A successful configuration of the coordinate system is achieved when the robot is near the center of the field before rotating in place.

From the center, the robot rotates in place until it detects the pool ball, while the distance sensor accounts for the proximity of the surrounding walls. Simultaneously, the gyroscope constantly records the heading of the robot. Instead of immediately travelling forwards upon the detection of the ball, additional steps were implemented to ensure the robot’s center aligns accurately with the center of the ball. This refinement operates when a ball is detected, with the robot advancing toward the ball if it lies at an excessive distance, and conversely, retreating backwards if it is positioned too close. Once the robot confirms that it is within an appropriate distance from the ball, it undergoes a positional adjustment. Using the distance sensor, it locates the ball’s endpoints, calculates the horizontal midpoint, and orients itself to point directly at the ball’s center. With the improved adjustment, the robot approaches the ball until it is approximately 5 cm away. The robot successfully completes these series of tasks when the ball’s center aligns with the brain’s center. 

Precedingly, the robot orbits around the ball without making any contact to ensure that the ball’s position remains unchanged. The angle of this orbital motion is calculated using the predefined coordinates of the pocket holes, the ball’s x and y positions, and the robot’s current heading which is based on the pocket closest to the ball. Once the robot is positioned at the optimal angle, it drives slightly forward to initiate the ball’s motion. This modification was introduced to overcome the ball’s inertia, since stationary objects require additional force to initiate motion, the force provided by the striker motor was insufficient. It then initializes the striker mechanism which launches the ball directly into the pocket. A successful pocketing attempt is denoted when the ball falls into the pocket or misses by ±5 cm from the target pocket.

Following each strike, a change was implemented to improve the robot’s heading by redefining the coordinate plane through scanning the two closest consecutive and perpendicular walls to realign with the table. The cycle to detect, navigate, orbit, and pocket the ball is repeated, starting with the rotation in place to scan for the missed ball. Lastly, the shutdown procedure occurs when the robot detects that there are no balls on the pool table, which then performs a celebratory spin and sound effect before exiting the program.


## Technology Stack
This project was developed using:
- **C++**: The primary programming language used to code the robot's functionality.
- **VEX IQ**: The platform used for building and controlling the robot, including motor control, sensor management, and communication.

## Mechanical Design 
The Pool-Playing robot design builds upon a Kiwi-drive base which uses custom 3D printed components to align the Vex parts. The chassis of the robot was designed with stability as a focus. The key element of the design is the striking mechanism which allows the robot to hit a ball in front of it with consistency and power. The brain is positioned directly above the centre such that it does not affect the movement dynamics of the robot, and it is easily accessible to change batteries or start the program. 
<div align="center">
  <img width="452" height="307" alt="CAD Finalized Robot Design" src="https://github.com/user-attachments/assets/32f63326-23b2-42e8-ab67-06828d87238c" />
  <p><i>Figure 1.1 CAD Finalized Robot Design</i></p>
</div>
<div align="center">
  <img width="459" height="367" alt="Physical Finalized Robot" src="https://github.com/user-attachments/assets/ad085a4d-a082-42d7-a8b5-7c76441268b6" />
  <p><i>Figure 1.2 Physical Finalized Robot</i></p>
</div>
<div align="center">
  <img width="481" height="177" alt="Finalized Striker Design" src="https://github.com/user-attachments/assets/80386009-ff30-4845-bc17-13a63e46598d" />
  <p><i>Figure 1.3 Finalized Striker Design</i></p>
</div>

## Software Design and Implementation
The software is broken into three main sections:
### 1) Start-up Procedure (Robot configuration)
  - **Sensor Calibration**: All sensors (distance sensor, gyroscope) are initialized.
  - **Striker Retracted**: Ensures the striker is pulled back.
  - **User Input**: Receives the starting corner information from the user through the controller.
  - **Navigates to Center**: Using motor encoders, the robot moves to the center of the pool table and sets it as the origin (0,0) in the Cartesian plane.
### 2) Pocketing the Pool Ball
  - **Ball Detection**: The robot rotates in place until the distance sensor locates the ball.
  - **Approach the Ball**: The robot moves towards the ball, stopping 120mm from it.
  - **Optimal Angle Calculation**: Calculates the optimal pocket angle based on the ball’s position and its shortest distane to a pocket.
  - **Orbit Around Ball**: The robot orbits around the ball without making contact to ensure it aligns for the shot.
  - **Strike the Ball**: The robot strikes the ball using the striker mechanism.
  - **Recalculate Position**: After a successful strike, the robot recalculates its position using the two closest perpendicular walls.
### 3) Shutdown Procedure
  - **Validation**: Checks that no ball is left on the pool table.
  - **Celebration**: If successful, the robot plays a celebratory sound and spins; otherwise, it repeats the pocketing process.
  - **Program Exit**: The robot closes the program after all balls are cleared.
<div align="center">
  <img width="452.8" height="760.8" alt="Flowchart of the Entire Program" src="https://github.com/user-attachments/assets/1bae400c-d621-4d47-95bd-cbd2a01d839b" />
  <p><i>Figure 2. Flowchart of the Entire Program</i></p>
</div>
