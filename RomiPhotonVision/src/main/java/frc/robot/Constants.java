// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

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

    // -------- Physical Constants -----------------
    public static final double kCountsPerRevolution = 1440.0;
    public static final double kWheelDiameterMeters = 0.07;
    public static final double kMetersPerDegree = Math.PI * 0.141 / 360;
    public static final double kTrackwidthMeters = 0.142072613;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    // Calibration for the right wheel voltage because it's much slower
    // than the left wheel on this robot.
    public static final double rightVoltsGain = 1.094;

    // Offset in the X and Y direction for the initial Pose
    public static final double yPoseOffset = 1.0;
    public static final double xPoseOffset = 0.25;

    // -------- Dynamical Constants --------------------

    // Max speed and acceleration of the robot
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelMetersPerSecondSquared = 0.5;

    // -------- PID Constants --------------------

    // Drive profile
    public static final TrapezoidProfile.Constraints kTrapezoidProfileConstraints =
        new TrapezoidProfile.Constraints(DriveConstants.kMaxSpeedMetersPerSecond,
                                        DriveConstants.kMaxAccelMetersPerSecondSquared);                                

    // For distances PID
    public static final double kPDriveVel = 0.5;
    // public static final double kIDriveVel = 0.1;
    public static final double kIDriveVel = 0;
    public static final double kDDriveVel = 0;

  }

  public final class ExtIOConstants {
    // Port configuration to match physical configuration on
    // Romi board as well as configuration on http://wpilib.local
    public static final int DIO0_PORT = 8;

    // The Romi has the left and right motors set to
    // PWM channels 0 and 1 respectively
    public static final int PWM2_PORT = 2;
    public static final int PWM3_PORT = 3;
    public static final int PWM4_PORT = 4;
  }

  public static final class VisionConstants {
    public static final double kP = -0.03;
    public static final double ForwardKP = 0.4;
    public static final double ForwardKD = 0.0;
    public static final double TurnKP = 0.01;
    
    // Constants such as camera and target height stored. Change per robot and goal!
    public static final double CAMERA_HEIGHT_METERS = 0.11;
    public static final double TARGET_HEIGHT_METERS = 0.215;

    // Angle between horizontal and the camera.
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0.0);

    // A Pose2d representing the target position in the field coordinate system.
    public static final Pose2d kFieldToEndTarget = new Pose2d(2.5, 1.0, new Rotation2d(0.0));
    public static final Pose2d kFieldToBeginTarget = new Pose2d(0.0, 1.0, new Rotation2d(180.0));
    public static final Pose2d kFieldToLeftTarget = new Pose2d(1.25, 2.0, new Rotation2d(90.0));
    public static final Pose2d kFieldToRightTarget = new Pose2d(1.25, 0.0, new Rotation2d(270.0));

    public static final Transform2d kCameraToRobot 
          = new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d());
  }  

  public final class ServoConstants {
    // Incremental amount of change for each button press
    // or during the periodic check while holding the button down
    public static final double SERVO_INCREMENT = 1.0;
  }

}

