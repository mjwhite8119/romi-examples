// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpiutil.math.Nat;

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

  // Pose estimator State Space way for tracking the robot pose
  private DifferentialDrivePoseEstimator m_estimator = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
        new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.

  // Show a field diagram for tracking Pose estimation
  private final Field2d m_estimatedField2d = new Field2d();
  
  // Also show a field diagram
  private final Field2d m_field2d = new Field2d();

  // Create a slew rate filter to give more control over the speed from the joystick
  private final SlewRateLimiter m_filter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter m_filter_turn = new SlewRateLimiter(0.5);
  
  // Used to put data onto Shuffleboard
  private ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain");

  private NetworkTableEntry m_heading = 
    driveTab.add("Current Heading", 0)
      .withPosition(5, 3)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();

  private final PIDController m_leftController =
    new PIDController(DriveConstants.kPDriveVel, 
                      DriveConstants.kIDriveVel, 
                      DriveConstants.kDDriveVel);

  private final PIDController m_rightController =
  new PIDController(DriveConstants.kPDriveVel, 
                    DriveConstants.kIDriveVel, 
                    DriveConstants.kDDriveVel);    

  /*********************************
   * Creates a new Drivetrain.
   * 
  */
  public Drivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * DriveConstants.kWheelDiameterMeters) / DriveConstants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * DriveConstants.kWheelDiameterMeters) / DriveConstants.kCountsPerRevolution);
    
    // Reset pose
    resetEncoders();
    resetGyro();

    // Call these to so that the information is immediatelly available in Simulator
    steer(0);
    arcadeDrive(0, 0);
    turn(0);

    // Setup Odometry
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    SmartDashboard.putData("field", m_field2d);

    // Setup Pose Estimator
    SmartDashboard.putData("fieldEstimate", m_estimatedField2d);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    SmartDashboard.putNumber("ArcadeDrive xaxisSpeed", xaxisSpeed);
    SmartDashboard.putNumber("ArcadeDrive zaxisRotate", zaxisRotate);
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void rateLimitedArcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(m_filter.calculate(xaxisSpeed), m_filter_turn.calculate(zaxisRotate));
  }

  /**
   * Tank drive method for differential drive platform. The calculated values 
   * will be squared to decrease sensitivity at low speeds.
   *
   * @param leftSpeed The robot's left side speed along the X axis [-1.0..1.0].
   * @param rightSpeed The robot's right side speed along the X axis [-1.0..1.0].
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    SmartDashboard.putNumber("PID Left Output", leftSpeed);
    SmartDashboard.putNumber("PID Right Output", rightSpeed);
    
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * 
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    
    // double rightVoltsCalibrated = rightVolts * DriveConstants.rightVoltsGain;
    SmartDashboard.putNumber("Total Left Volts", leftVolts);
    SmartDashboard.putNumber("Total Right Volts", rightVolts);

    // Apply the voltage to the wheels
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(-rightVolts); // We invert this to maintain +ve = forward
    m_diffDrive.feed();
  }

  /**
   * Drives a straight line at the requested velocity by applying feedforward
   * and PID output to maintain the velocity. This method calculates a voltage
   * value for each wheel, which is sent to the motors setVoltage() method.
   * 
   * @param velocity The velocity at which to drive
   */
  public void steerVelocity(double velocity) {
    SmartDashboard.putNumber("Requested Velocity", velocity);

    // Calculate feedforward voltage
    double leftFeedforward = DriveConstants.kLeftFeedForward.calculate(velocity);
    double rightFeedforward = DriveConstants.kRightFeedForward.calculate(velocity);
    SmartDashboard.putNumber("Left Feedforward Volts", leftFeedforward);
    SmartDashboard.putNumber("Right Feedforward Volts", rightFeedforward);

    // Send it through a PID controller
    double leftVelocity = m_leftController.calculate(m_leftEncoder.getRate(), velocity);
    double rightVelocity = m_rightController.calculate(m_rightEncoder.getRate(), velocity);
    SmartDashboard.putNumber("Left Volts", leftVelocity);
    SmartDashboard.putNumber("Right Volts", rightVelocity);

    // double calibratedRightSpeed = output * DriveConstants.rightVoltsGain;
    tankDriveVolts(leftFeedforward + leftVelocity, rightFeedforward + rightVelocity);
  }

  /**
   * Drives a straight line at the requested output -1.0..1.0
   * 
   * @param output Output value between -1.0..1.0
   */
  public void steer(double output) {
    SmartDashboard.putNumber("Requested Output", output);

    // double calibratedRightSpeed = output * DriveConstants.rightVoltsGain;
    tankDrive(output, output);
  }

  /**
   * Drives a straight line at the requested output -1.0..1.0
   * 
   * @param output Output value between -1.0..1.0
   */
  public void driveLine(double rotate) {

    // Restrict the turn speed
    double zRotation = MathUtil.clamp(rotate, -0.5, 5.0);

    // Slow down around tight bends
    double speed = 0.5 - (Math.abs(zRotation) * 0.05);
    arcadeDrive(speed, zRotation);
  }

  public void turn(double output) {
    // Restrict the turn speed
    double zRotation = MathUtil.clamp(output, -0.5, 5.0);
    arcadeDrive(0, zRotation); 
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
    double leftDistance = getLeftDistanceMeters();
    double rightDistance = getRightDistanceMeters();
    SmartDashboard.putNumber("Left distance", leftDistance);
    SmartDashboard.putNumber("Right distance", rightDistance);
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
    publishTelemetry();
  }

  /**  
   * Publishes telemetry data to the Network Tables for use
   * in Shuffleboard and the Simulator
  */
  public void publishTelemetry() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    
    // Offset the pose to start 1.5 meters on the Y axis
    double yPoseOffset = 1.5;
    Pose2d currentPose = getPose();
    Pose2d poseOffset = new Pose2d(currentPose.getX(), 
                                   currentPose.getY() + yPoseOffset, 
                                   currentPose.getRotation());
    // Update the Field2D object (so that we can visualize this in sim)
    m_field2d.setRobotPose(poseOffset);

    // Updates the the Unscented Kalman Filter using only wheel encoder information.
    m_estimator.update(m_gyro.getRotation2d(), 
                      getWheelSpeeds(), 
                       m_leftEncoder.getDistance(), 
                       m_rightEncoder.getDistance());


    // Offset the pose to start 1.5 meters on the Y axis
    Pose2d currentEstimatedPose = getEstimatedPose();
    Pose2d estimatedPoseOffset = new Pose2d(currentEstimatedPose.getX(), 
                                            currentEstimatedPose.getY() + yPoseOffset, 
                                            currentEstimatedPose.getRotation());

    // Update the Field2D object (so that we can visualize this in sim)
    m_estimatedField2d.setRobotPose(estimatedPoseOffset);

    // Display the meters per/second for each wheel and the heading
    SmartDashboard.putNumber("Left Encoder Velocity", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Encoder Velocity", m_rightEncoder.getRate());
    SmartDashboard.putNumber("Heading", getHeading());
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
  public void resetOdometry() {
    resetEncoders();
    resetGyro();
    // m_odometry.resetPosition(pose, m_gyro.getRotation2d());
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

  /**
   * Returns the currently estimated pose of the robot.
   * @return The pose
   */
  public Pose2d getEstimatedPose() {
    return m_estimator.getEstimatedPosition();
  }

  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

}
