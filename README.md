# 2024 Offseason Robot Code
The purpose of this repo is to test new software with the existing 2024 comp robot, Khaos. The reason for creating a new repo and not branch is because the entire code will be revamped and it is cleaner to start fresh.

## Changes Made
- [x] Autoformat
- [X] Advantage Kit
- [x] CTRE Swerve Generator
- [x] Updated Commands trees
- [x] Better Kalman Filters for vision
- [x] Driver Automation regarding shooting

## Software Features of the Robot
- Around 70% of robot functions automated (Operator role eliminated, only 1 driver required)
- Robot knows where it is on the field through vision system for tracking fiducials on the field and updating odometry
- Swerve Drive 250hz odometry for fast sensor fusion and update rates
- Computer vision uses an AI Yolo v8 model to track game pieces and automatically drive and intake them without human intervention
- Subsystems are all replay and physics simulation compatible through hardware abstraction
- Logging almost every hardware to debug issues easily
- Commands involving individual subsystems converted to state machines
- Automatic distance based shooting based off of pose determined by fiducial tracking

## AI Game Piece Detection
https://github.com/user-attachments/assets/f61318d6-b2fd-4a13-a274-39e5f386329a

## Fiducial-based Distance Shooting
https://github.com/user-attachments/assets/6f8c3180-7268-4ef0-b97a-91d17584ffd3

## Automatic Formatting with `spotless`

### Check Formatting

To check the formatting, simply run the following commmand
depending on your operating system.

Windows: `.\gradlew spotlessCheck`

MacOS/Linux: `gradlew spotlessCheck`

### Apply Formatting

To apply formatting, simply run the following commmand
depending on your operating system.

Windows: `.\gradlew spotlessApply`

MacOS/Linux: `gradlew spotlessApply`

### Building

Whenever the project builds, the format is automatically applied.
Note that, whenever you deploy the project, it builds before the code is actually sent to the robot so the formatting will be applied on a deploy as well.

### `spotless` GitHub Actions

Whenever you make a pull request (PR), GitHub will run a formatting check on the code to see if it is formatted or not.
If the code is not formatted, it will fail the tests. If it fails 
the formatting tests, make sure to format it and commit the 
formatted code (using the above methods) and add it to the pull 
request.

For an example of what the output should look like when the tasks
suceeed, see [this example output](https://github.com/Robohawks-346-Programming/Team346_2024_Offseason/actions/runs/9192787141).
The first tasks it runs, the `format` tasks, checks the formatting
of the code. The second task it runs, the `build` task, builds the code on a Linux server that emulates a RoboRIO.
