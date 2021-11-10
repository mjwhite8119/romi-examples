# Romi Basic Arm
This project uses the [Robot Arm kit for Romi](https://www.pololu.com/product/3550) from Pololu. It starts with the [DrivetrainBase](https://github.com/mjwhite8119/romi-examples/tree/main/DrivetrainBase) project that provides a fully functional Romi drivetrain, and adds commands to control the arm.

The project also assumes that you have characterized your Romi using the FRC Charaterization Tool since there might be slight variations between Romis (due to assembly, mechanical difference, etc).   To run characterization on your Romi download the [Romi characterization program](https://github.com/bb-frc-workshops/romi-examples/tree/main/romi-characterization) on the **bb-frc-workshops** GitHub account.  You can use the values provided in this project to start with but running the characterization will give you better results. 

Note that all the constants used here assume that characterization has been done using meters as units. Additionally, all coordinates/distances are specified in meters.

The following commands have been created to demonstrate the use of the Romi Arm.

