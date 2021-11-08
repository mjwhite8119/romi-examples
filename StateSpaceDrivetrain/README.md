# Romi State Space Drive
This project uses the Romi to demonstrate the use State Space Control. It starts with the [DrivetrainBase](https://github.com/mjwhite8119/romi-examples/tree/main/DrivetrainBase) project that provides a fully functional Romi drivetrain, and adds commands with the goal of driving the Romi on a straight line path. 

The project also assumes that you have characterized your Romi using the FRC Charaterization Tool since there might be slight variations between Romis (due to assembly, mechanical difference, etc).   To run characterization on your Romi download the [Romi characterization program](https://github.com/bb-frc-workshops/romi-examples/tree/main/romi-characterization) on the **bb-frc-workshops** GitHub account.  You can use the values provided in this project to start with but running the characterization will give you better results. 

Note that all the constants used here assume that characterization has been done using meters as units. Additionally, all coordinates/distances are specified in meters.

The following commands and methods have been created to demonstrate the use of State Space control.

- **StateSpaceDrive** - 