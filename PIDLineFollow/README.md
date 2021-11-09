# Romi PID Line Follow
This project has the Romi follow a line marked on the floor. It starts with the [DrivetrainBase](https://github.com/mjwhite8119/romi-examples/tree/main/DrivetrainBase) project that provides a fully functional Romi drivetrain, and adds Vision processes and PID commands with the goal of following the line. 

The project also assumes that you have characterized your Romi using the FRC Charaterization Tool since there might be slight variations between Romis (due to assembly, mechanical difference, etc).   To run characterization on your Romi download the [Romi characterization program](https://github.com/bb-frc-workshops/romi-examples/tree/main/romi-characterization) on the **bb-frc-workshops** GitHub account.  You can use the values provided in this project to start with but running the characterization will give you better results. 

Note that all the constants used here assume that characterization has been done using meters as units. Additionally, all coordinates/distances are specified in meters.

The following classes and commands have been created.

- A python vision program that gets uploaded to the Romi to extract objects of interest from the camera stream.  This program will put relevant data into the Network Tables so as the PIDLineFollow program can use it to follow the line.  This python program resides in the *Vision* folder of this project.

- **Vision** - This class gets data from the Network Tables that was put there by the python vision program running on the Romi.

- **LineFollowPIDCommand** - Follows the line using the x-center line data from the Vision class.