// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// New classes for this project
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

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

  // Identify a standard differential drive drivetrain, given the drivetrain's kV and kA in both
  // linear (volts/(meter/sec) and volts/(meter/sec^2)) and angular (volts/(radian/sec) and
  // volts/(radian/sec^2)) cases. 
  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  // 
  // The plant holds a state-space model of our drivetrain. This system has the following properties:
  // 
  // State is: [left velocity, right velocity]
  // Inputs are [left voltage, right voltage]
  // Outputs are [left velocity, right velocity].
  
  private final LinearSystem<N2, N2, N2> m_drivetrainPlant =
      LinearSystemId.identifyDrivetrainSystem(Constants.DriveConstants.kvVoltSecondsPerMeter, 
                                              Constants.DriveConstants.kaVoltSecondsSquaredPerMeter, 
                                              Constants.DriveConstants.kvVoltSecondsPerRadian, 
                                              Constants.DriveConstants.kaVoltSecondsSquaredPerRadian);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N2, N2, N2> m_observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N2(),
          m_drivetrainPlant,
          VecBuilder.fill(3.0, 3.0), // How accurate we think our model is
          VecBuilder.fill(0.05, 0.05), // How accurate we think our encoder
          // data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N2, N2, N2> m_controller =
      new LinearQuadraticRegulator<>(
          m_drivetrainPlant,
          VecBuilder.fill(0.2, 0.2), // qelms. Velocity
          // error tolerances, in meters per second. Decrease this to more
          // heavily penalize state excursion, or make the controller behave more aggressively.
          VecBuilder.fill(6.0, 6.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 6 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020);

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  public final LinearSystemLoop<N2, N2, N2> m_loop =
      new LinearSystemLoop<>(m_drivetrainPlant, m_controller, m_observer, Constants.DriveConstants.maxVolts, 0.020);

  // Used to put data onto Shuffleboard
  private ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain");
  private NetworkTableEntry m_leftEncoderRate = 
    driveTab.add("Left Encoder Rate", getLeftEncoderRate())
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(3, 3)
      .getEntry();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * Constants.DriveConstants.kWheelDiameterMeter) / Constants.DriveConstants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * Constants.DriveConstants.kWheelDiameterMeter) / Constants.DriveConstants.kCountsPerRevolution);
    resetEncoders();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void setLeftVoltage(double voltage) {
    m_leftMotor.setVoltage(voltage);
  }

  public void setRightVoltage(double voltage) {
    m_rightMotor.setVoltage(voltage);
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

  // Get the current rate of the encoder. 
  // Units are distance per second (speed) as
  // scaled by the value from setDistancePerPulse().
  // Also write out to the network tables for Shuffleboard
  public double getLeftEncoderRate() {
    double rate = m_leftEncoder.getRate();
    m_leftEncoderRate.setDouble(rate);
    return rate;
  }

  public double getRightEncoderRate() {
    return m_rightEncoder.getRate();
  }

  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeter() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceMeter() {
    return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
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
    // This method will be called once per scheduler run
  }
}
