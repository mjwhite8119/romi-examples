// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {

        // -------- Physical Constants -----------------
        public static final double kCountsPerRevolution = 1440.0;
        public static final double kWheelDiameterMeters = 0.07; // 70 mm
        public static final double kTrackwidthMeters = 0.142072613;
        public static final double kMetersPerDegree = Math.PI * 0.141 / 360;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        // Calibration for the right wheel voltage because it's much slower
        // than the left wheel on this robot.
        public static final double rightVoltsGain = 1.03;

        // -------- Dynamical Constants --------------------

        // Max speed and acceleration of the robot
        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAccelMetersPerSecondSquared = 0.5;

        public static final double kMaxTurnRateDegPerS = 360;
        public static final double kMaxTurnAccelDegPerSSquared = 200;

        // The linear inertia gain, volts
        public static final double ksVolts = 0.461;
        // The linear velocity gain, volts per (meter per second)
        // Increase this if you drive short
        public static final double kvVoltSecondsPerMeter = 6.93;
        public static final double kvVoltSecondsPerMeterLeft = 8.8;
        public static final double kvVoltSecondsPerMeterRight = 9.5;
        // The linear acceleration gain, volts per (meter per second squared).
        public static final double kaVoltSecondsSquaredPerMeter = 0.0737;

        // Setup constraints for feedforward and kinematics
        public static final SimpleMotorFeedforward kFeedForward = 
            new SimpleMotorFeedforward(ksVolts, 
                                        kvVoltSecondsPerMeter, 
                                        kaVoltSecondsSquaredPerMeter);
                                        
        // Left and Right motors are very different, so each has its own FF.
        public static final SimpleMotorFeedforward kLeftFeedForward = 
            new SimpleMotorFeedforward(ksVolts, 
                                        kvVoltSecondsPerMeterLeft, 
                                        kaVoltSecondsSquaredPerMeter);

        public static final SimpleMotorFeedforward kRightFeedForward = 
            new SimpleMotorFeedforward(ksVolts, 
                                        kvVoltSecondsPerMeterRight, 
                                        kaVoltSecondsSquaredPerMeter);

        public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                kFeedForward,
                kDriveKinematics,
                10);

        // -------- PID Constants --------------------

        // Drive profile
        public static final TrapezoidProfile.Constraints kTrapezoidProfileConstraints =
            new TrapezoidProfile.Constraints(DriveConstants.kMaxSpeedMetersPerSecond,
                                            DriveConstants.kMaxAccelMetersPerSecondSquared);

        // Turn profile                                    
        public static final TrapezoidProfile.Constraints kTrapezoidProfileTurnConstraints =
            new TrapezoidProfile.Constraints(DriveConstants.kMaxTurnRateDegPerS,
                                             DriveConstants.kMaxTurnAccelDegPerSSquared);                                    

        // For distances PID
        public static final double kPDriveVel = 2;
        public static final double kIDriveVel = 0.1;
        public static final double kDDriveVel = 0;

        // Tolerances for distance drive
        public static final double kDistanceToleranceMeters = 0.05;
        public static final double kVelocityToleranceMetersPerS = 0.05;

        // For turns PID
        public static final double kPTurnVel = 0.05;
        public static final double kITurnVel = 0;
        public static final double kDTurnVel = 0.005;

        // For profiled turns PID
        public static final double kPTurnVelProfiled = 0.05;
        public static final double kITurnVelProfiled = 0;
        public static final double kDTurnVelProfiled = 0;

        // Tolerances for turns
        public static final double kTurnToleranceDeg = 5.0;
        public static final double kTurnRateToleranceDegPerS = 10.0;
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

    public final class Arm {
        // Port configuration to match physical configuration on
        // Romi board as well as configuration on http://wpilib.local
        public static final int LIFT_PORT = 4;
        public static final int TILT_PORT = 3;
        public static final int GRIPPER_PORT = 2;
        public static final int GRIPPER_FEEDBACK_PORT = 0;

        // From https://www.pololu.com/docs/0J76/4
        // Servo ranges are already restricted in the core Servo
        // class to 600 - 2400 microseconds. However, the
        // arm does have the ability to bind if the servos move past 
        // the recommended ranges. With that, we are further restricting
        // the range of our arm servos. Adjust these for more range of motion
        // but do so with caution.
        // Lift range 1000 - 1900 -> .222 - .722
        // Tilt range 1200 - 1900 -> .333 - .722
        // Gripper range 500 - 2400 -> 0 - 1
        public static final double LIFT_MIN = 0.222;
        // public static final double LIFT_MIN = 0.550;
        public static final double LIFT_MAX = 0.722;
        // public static final double TILT_MIN = 0.697;
        public static final double TILT_MIN = 0.333;
        public static final double TILT_MAX = 0.722;
        public static final double GRIPPER_MIN = 0.0;
        public static final double GRIPPER_MAX = 0.86;

        // Incremental amount of change for each button press
        // or during the periodic check while holding the button down
        public static final double SERVO_INCREMENT = 0.005;
    }
}
