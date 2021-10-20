// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // public static final double ksVolts = 0.929;
        // public static final double kvVoltSecondsPerMeter = 6.33;
        // public static final double kaVoltSecondsSquaredPerMeter = 0.0389;
        public static final double ksVolts = 0.461;

        // The linear velocity gain, volts per (meter per second)
        public static final double kvVoltSecondsPerMeter = 9.7;
        // The angular velocity gain, volts per (radians per second)
        public static final double kvVoltSecondsPerRadian = 0.345;

        // The linear acceleration gain, volts per (meter per second squared).
        public static final double kaVoltSecondsSquaredPerMeter = 0.0737;
        // The angular acceleration gain, volts per (radians per second squared)
        public static final double kaVoltSecondsSquaredPerRadian = 0.00235;

        // Max volts that can be sent to the motors
        public static final double maxVolts = 6.0;

        public static final double kPDriveVel = 0.085;
        // public static final double kPDriveVel = 0.141;

        public static final double kCountsPerRevolution = 1440.0;
        public static final double kWheelDiameterMeter = 0.07;

        public static final double maxVelocityPerSecond = 0.5;
        public static final double maxAccelPerSecond = 1.0;

        public static final double kTrackwidthMeters = 0.142072613;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        
    }

    public final class Joystick {
        // Button mapping for a PS3 gamepad. 
        public static final int SELECT = 1;
        public static final int A = 2;
        public static final int B = 3;
        public static final int START = 4;
        public static final int TOP_DIR = 5;
        public static final int RIGHT_DIR = 6;
        public static final int BOTTOM_DIR = 7;
        public static final int LEFT_DIR = 8;
        public static final int L2_BUTTON = 9;
        public static final int R2_BUTTON = 10;
        public static final int LEFT_ANALOG = 11;
        public static final int RIGHT_ANALOG = 12;
        public static final int TRIANGLE_BUTTON = 13;
        public static final int CIRCLE_BUTTON = 14;
        public static final int CROSS_BUTTON = 15;
        public static final int SQUARE_BUTTON = 16;
        public static final int UNKNOWN_BUTTON = 17;
    }
}
