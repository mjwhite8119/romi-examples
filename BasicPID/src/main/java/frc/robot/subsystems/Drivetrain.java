// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  
  // Also show a field diagram
  private final Field2d m_field2d = new Field2d();
  
  // Used to put data onto Shuffleboard
  private ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain");

  private NetworkTableEntry m_heading = 
    driveTab.add("Current Heading", 0)
      .withPosition(5, 3)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * DriveConstants.kWheelDiameterMeters) / DriveConstants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * DriveConstants.kWheelDiameterMeters) / DriveConstants.kCountsPerRevolution);
    resetEncoders();
    resetGyro();

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    SmartDashboard.putData("field", m_field2d);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    SmartDashboard.putNumber("zaxisRotate", zaxisRotate);
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void steer(double speed) {
    SmartDashboard.putNumber("speed", speed);
    arcadeDrive(speed, 0);
  }

  public void turn(double rotate) {
    if (rotate > 0.4) {
      arcadeDrive(0, 0.4);
    } else if (rotate < -0.4) {
      arcadeDrive(0, -0.4);  
    } else {
      arcadeDrive(0, rotate);
    }   
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    
    // Also update the Field2D object (so that we can visualize this in sim)
    m_field2d.setRobotPose(getPose());

    m_heading.setDouble(getHeading());
    SmartDashboard.putNumber("Position", getAverageDistanceMeters());
  }

  /**
   * Returns the currently estimated pose of the robot.
   * @return The pose
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Zeroes the heading of the robot
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    
    // double angle = getGyroAngleZ();
    // double rotations = Math.round(angle/360);
    // double heading = angle - (rotations*360);
    double heading = m_gyro.getRotation2d().getDegrees();
    return heading;
  }

}
