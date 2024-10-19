# Obstacle Avoidance - Innovation Challenge 2024 - UofM
Fastest fully autonomous code to control freenove in obstacle avoidance course 

How It Works:

    Move Forward: Robot moves straight until an obstacle is detected (within 30 cm).
    Scan for Best Path:
        Servo sweeps sonar between 30° and 150°.
        Chooses the direction with the furthest distance.
    Turn and Align:
        Robot dynamically turns toward the chosen path.
    Pass the Obstacle:
        Continues forward while monitoring to detect when the object has been passed.
    Realign: Reverses turn and aligns back to the original path.
    Repeat: Loops through the process for new obstacles.

Key Features:

    Sweep Scanning: Servo-mounted sensor scans and detects obstacles.
    Dynamic Turning: Robot turns in real-time based on best path detection.
    Pass Detection: Detects when the robot passes an obstacle and adjusts.
    Speed Compensation: Adjusts speed based on battery voltage.
