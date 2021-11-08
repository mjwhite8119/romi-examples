# Romi DrivetrainBase

This project takes the WPILib [romiReference](https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/romireference) example project and enhances it with methods, classes, and constants to form the basis for more advanced projects.  It includes code to view the Pose of the drive train in the Simulator and has the *RomiGyro* class implement the WPILib *Gyro* class to provided additional functionality. Teams would probably go on to charaterize their robot using the FRC [Robot Charactization Tool](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/index.html) before implementing PID, Trajectories, and State Space Control.

Here's a list of the specific enhancements:

- Changes all measurements to meters.

- Adds the *DifferentialDriveKinematics* class to the *Constants* file. 

- Updates the *RomiGyro* subsystem to implement Gyro.

- Adds `getHeading()` method to Drivetrain using the `getRotation2d()` method that was made available when we updated *RomiGyro*.

- Added methods to get the encoder rates (meters per/second).

- Includes *DifferentialDriveWheelSpeeds* class.

- Adds the *DifferentialDriveOdometry* class to *Drivetrain* together with a Field2D object for viewing the robot's in the Simulator. This is updated in the Drivetrain's `periodic()` method.

- Adds the *DifferentialDrivePoseEstimator* class to *Drivetrain* together with a Field2D object for viewing the robot's pose in the Simulator. This is updated in the Drivetrain's `periodic()` method.

- Implements a *SlewRateLimiter* filter to more smoothly control inputs to the motors.

- Adds a method to write telemetry data to the Network Tables for use in Shuffleboard and the Simulator.  This method is called from the Drivetrain's `periodic()` method.