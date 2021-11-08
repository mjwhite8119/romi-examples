# Romi Trajectory Follower
This example showcases how to use the WPILib *RamseteCommand* to make your Romi follow a predefined trajectory. These trajectories can be hand crafted, or generated using a tool like PathWeaver. It starts with the [DrivetrainBase](https://github.com/mjwhite8119/romi-examples/tree/main/DrivetrainBase) project that provides a fully functional Romi drivetrain.  The original *RamseteCommand* example came from **bb-frc-workshops** [Romi trajectory example](https://github.com/bb-frc-workshops/romi-examples/tree/main/romi-trajectory-ramsete).


The project also assumes that you have characterized your Romi using the FRC Charaterization Tool since there might be slight variations between Romis (due to assembly, mechanical difference, etc).   To run characterization on your Romi download the [Romi characterization program](https://github.com/bb-frc-workshops/romi-examples/tree/main/romi-characterization) on the **bb-frc-workshops** GitHub account.  You can use the values provided in this project to start with but running the characterization will give you better results. 

Note that all the constants used here assume that characterization has been done using meters as units. Additionally, all coordinates/distances are specified in meters.

## Additional Hardware Required
None

## Additional Configuration Required
- Ensure that the gyro has been [calibrated using the web UI](https://docs.wpilib.org/en/stable/docs/romi-robot/web-ui.html#imu-calibration)


## Additional Code Setup
The trajectory can be modified by editing the `generateRamseteCommand` method in `RobotContainer.java`