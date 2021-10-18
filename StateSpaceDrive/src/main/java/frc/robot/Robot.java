// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * This is a sample program to demonstrate how to use a state-space controller to control a
 * flywheel.
 */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;
  private static final int kJoystickPort = 0;
  private static final double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(500.0);

  // The linear velocity gain, volts per (meter per second)
  private static final double kVLinear = 6.33;
  // The angular velocity gain, volts per (radians per second)
  private static final double kVAngular = 3.33;

  // The linear acceleration gain, volts per (meter per second squared).
  private static final double kALinear = 0.0135;
  // The angular acceleration gain, volts per (radians per second squared)
  private static final double kAAngular = 0.007;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(
          Units.feetToMeters(3.0), Units.feetToMeters(6.0)); // Max drivetrain speed and acceleration.
  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  // Identify a standard differential drive drivetrain, given the drivetrain's kV and kA in both
  // linear (volts/(meter/sec) and volts/(meter/sec^2)) and angular (volts/(radian/sec) and
  // volts/(radian/sec^2)) cases. 
  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  // 
  // The plant holds a state-space model of our drivetrain. This system has the following properties:
  // 
  // [left velocity, right velocity]^T, inputs are [left voltage, right voltage]^T, and
  // outputs are [left velocity, right velocity]^T.
  
  private final LinearSystem<N2, N2, N2> m_drivetrainPlant =
      LinearSystemId.identifyDrivetrainSystem(kVLinear, kALinear, kVAngular, kAAngular);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N2, N2, N2> m_observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N2(),
          m_drivetrainPlant,
          VecBuilder.fill(3.0, 3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N2, N2, N2> m_controller =
      new LinearQuadraticRegulator<>(
          m_drivetrainPlant,
          VecBuilder.fill(8.0), // Velocity error tolerance
          VecBuilder.fill(6.0), // Control effort (voltage) tolerance
          0.020);

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N2, N2, N2> m_loop =
      new LinearSystemLoop<>(m_drivetrainPlant, m_controller, m_observer, 12.0, 0.020);

  // An encoder set up to measure flywheel velocity in radians per second.
  private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);

  private final SpeedController m_motor = new PWMSparkMax(kMotorPort);

  // A joystick to read the trigger from.
  private final Joystick m_joystick = new Joystick(kJoystickPort);

  @Override
  public void robotInit() {
    // We go 2 pi radians per 4096 clicks.
    // m_encoder.setDistancePerPulse(2.0 * Math.PI / 4096.0);
    // m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    // m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
  }

  @Override
  public void teleopInit() {
    // Reset our loop to make sure it's in a known state.
    m_loop.reset(Matrix.mat(Nat.N2(), Nat.N1()).fill(0,0));
  }

  @Override
  public void teleopPeriodic() {

    // Sets the target speed of our flywheel. This is similar to setting the setpoint of a
    // PID controller.
    if (m_joystick.getTriggerPressed()) {
      // We just pressed the trigger, so let's set our next reference
      m_loop.setNextR(VecBuilder.fill(kSpinupRadPerSec));
    } else if (m_joystick.getTriggerReleased()) {
      // We just released the trigger, so let's spin down
      m_loop.setNextR(VecBuilder.fill(0.0));
    }

    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(m_encoder.getRate()));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    m_motor.setVoltage(nextVoltage);
  }
}
