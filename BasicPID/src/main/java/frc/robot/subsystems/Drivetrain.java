// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
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
    m_leftEncoder.setDistancePerPulse((Math.PI * Constants.DriveConstants.kWheelDiameterMeters) / Constants.DriveConstants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * Constants.DriveConstants.kWheelDiameterMeters) / Constants.DriveConstants.kCountsPerRevolution);
    resetEncoders();
    resetGyro();
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
    arcadeDrive(0, rotate);
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

  public double getLeftDistance() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceMeters() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
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

  public double getHeading() {
    double angle = getGyroAngleZ();
    double rotations = Math.round(angle/360);
    double heading = angle - (rotations*360);
    return heading;
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_heading.setDouble(getHeading());
  }
}
