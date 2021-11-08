# Romi PID Control
This project uses the Romi to demonstrate the use of PID controllers. It starts with the [DrivetrainBase](https://github.com/mjwhite8119/romi-examples/tree/main/DrivetrainBase) project that provides a fully functional Romi drivetrain, and adds PID commands with the goal of driving the Romi on a square path. 

The project also assumes that you have characterized your Romi using the FRC Charaterization Tool since there might be slight variations between Romis (due to assembly, mechanical difference, etc).   To run characterization on your Romi download the [Romi characterization program](https://github.com/bb-frc-workshops/romi-examples/tree/main/romi-characterization) on the **bb-frc-workshops** GitHub account.  You can use the values provided in this project to start with but running the characterization will give you better results. 

Note that all the constants used here assume that characterization has been done using meters as units. Additionally, all coordinates/distances are specified in meters.

The following commands have been created to demonstrate the use of PID control.

- **DriveDistancePID** - Uses a *PIDCommand* with a *PIDController* to drive a requested distance. 

- **DriveDistanceProfiled** - Uses a *ProfiledPIDCommand* with a *ProfiledPIDController* to drive a requested distance. 

- **TurnToAnglePID** - Uses a *PIDCommand* with a *PIDController* to turn to a requested angle. 

- **TurnToAngleProfiled** - Uses a *ProfiledPIDCommand* with a *ProfiledPIDController* to turn to a requested angle.

- **AutonomousPIDDistance** - Uses a *SequentialCommandGroup* to drive the Romi on a square path.

## PID Command Outputs
The nice thing about these commands is that you get to decide what to do with the output.  You can send it straight to the `arcadeDrive()` method of *DifferentialDrive* class or create other methods to give you more control. A method in the Drivetrain called `steerVelocity()` is used to drives a straight line at the requested velocity by applying feedforward and PID output to maintain the velocity. This method calculates a voltage value for each wheel, which is sent to the motors `setVoltage()` method.