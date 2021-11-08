# Romi PID Control
This project uses the Romi to demonstrate the use of PID controllers. It starts with the [DrivetrainBase](https://github.com/mjwhite8119/romi-examples/tree/main/DrivetrainBase) project that provides a fully functional Romi drivetrain.  The project also assumes that you have characterized your Romi using the FRC Charaterization Tool.  If you haven't done that then you can use the values provided in this project.  The following commands have been created to demonstrate the use of PID control.

- *DriveDistancePID* - Uses a *PIDCommand* with a *PIDController* to drive a requested distance. 

- *DriveDistanceProfiled* -
- *TurnToAnglePID* -
- *TurnToAngleProfiled* - 

## PID Command Outputs
The nice thing about these commands is that you get to decide what to do with the output.  You can send it straight to the `arcadeDrive()` method of *DifferentialDrive* class or create other methods to give you more control. A method in the Drivetrain called `steerVelocity()` is used to drives a straight line at the requested velocity by applying feedforward and PID output to maintain the velocity. This method calculates a voltage value for each wheel, which is sent to the motors `setVoltage()` method.