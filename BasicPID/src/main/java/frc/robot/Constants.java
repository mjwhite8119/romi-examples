// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class DriveConstants {
        public static final double kCountsPerRevolution = 1440.0;
        public static final double kWheelDiameterInch = 2.75591; // 70 mm
        public static final double kWheelDiameterMeters = 0.07; // 70 mm

        // For distances
        public static final double kDistanceP = 2.5;
        public static final double kDistanceI = 0;
        public static final double kDistanceD = 0;

        public static final double kDistanceToleranceMeters = 0.1;
        public static final double kVelocityToleranceMetersPerS = 1;

        // public static final double kMaxVelocityMeters = 0.15;
        // public static final double kMaxAcceMeters = 1.8;
        public static final double kMaxSpeedMetersPerSecond = 0.7;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.7;

        // For turns
        public static final double kTurnP = 1;
        public static final double kTurnI = 0;
        public static final double kTurnD = 0;

        public static final double kTurnToleranceDeg = 5;
        public static final double kTurnRateToleranceDegPerS = 10;
    }
}
