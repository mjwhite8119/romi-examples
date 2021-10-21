// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpilibj.util.Units;

public class Arm extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Servo m_lift = new Servo(Constants.Arm.LIFT_PORT);
  private final Servo m_tilt = new Servo(Constants.Arm.TILT_PORT);
  private final Servo m_gripper = new Servo(Constants.Arm.GRIPPER_PORT);
  private final AnalogInput m_gripperRead = new AnalogInput(Constants.Arm.GRIPPER_FEEDBACK_PORT);
  private double m_liftPos;
  private double m_tiltPos;
  private double m_gripperPos;

  // The plant holds a state-space model of our arm. This system has the following properties:
  //
  // States: [position, velocity], in radians and radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [position], in radians.
  // This won't work for servos...
  private final LinearSystem<N2, N1, N1> m_armPlant =
      LinearSystemId.identifyPositionSystem(0,0);


  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N2, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N1(),
          m_armPlant,
          VecBuilder.fill(0.015, 0.17), // How accurate we
          // think our model is, in radians and radians/sec
          VecBuilder.fill(0.01), // How accurate we think our encoder position
          // data is. In this case we very highly trust our encoder position reading.
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N2, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_armPlant,
          VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)), // qelms.
          // Position and velocity error tolerances, in radians and radians per second. Decrease
          // this
          // to more heavily penalize state excursion, or make the controller behave more
          // aggressively. In this example we weight position much more highly than velocity, but
          // this
          // can be tuned to balance the two.
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N2, N1, N1> m_loop =
      new LinearSystemLoop<>(m_armPlant, m_controller, m_observer, 12.0, 0.020);

  // Reset position to resting state
  public void reset() {
    m_liftPos = 0.5;
    m_tiltPos = 0.5;
    m_gripperPos = 0.5;

    m_lift.set(m_liftPos);
    m_tilt.set(m_tiltPos);
    m_gripper.set(m_gripperPos);
  }

  /** 
   * Increment tilt motor position
   * 
   * @param delta Amount to change motor position
   */
  public void incrementTilt(double delta) {
    m_tiltPos = saturateLimit(m_tiltPos + delta, Constants.Arm.TILT_MIN, Constants.Arm.TILT_MAX);
    m_tilt.set(m_tiltPos);
    // System.out.println("Tilt " + m_tiltPos);
  }

  /**
   * Increment lift motor position
   * 
   * @param delta Amount to change motor position
   */
  public void incrementLift(double delta) {
    m_liftPos = saturateLimit(m_liftPos + delta,  Constants.Arm.LIFT_MIN,  Constants.Arm.LIFT_MAX); 
    m_lift.set(m_liftPos);
    // System.out.println("Lift " + m_liftPos);
  }

  /** 
   * Increment gripper motor position
   * 
   * @param delta Amount to change motor position
   */ 
  public void incrementGripper(double delta) {
    m_gripperPos = saturateLimit(m_gripperPos + delta,  Constants.Arm.GRIPPER_MIN,  Constants.Arm.GRIPPER_MAX); 
    m_gripper.set(m_gripperPos);
    // System.out.println("Gripper " + m_gripperPos);
  }

  // Get lift motor position 
  public double getLiftPos() {
    return m_liftPos;
  }

  // Get tilt motor position 
  public double getTiltPos() {
    return m_tiltPos;
  }

  // Get lift motor position 
  public double getGripperPos() {
    return m_gripperPos;
  }

  // Get gripper motor position from feedback signal
  public int getGripperFeedbackPos() {
    return m_gripperRead.getAverageValue();
  }

  // Limit motor range to avoid moving beyond safe ranges
  public double saturateLimit(double val, double l_limit, double u_limit) {
    double outval = val;
    if(val > u_limit) {
      outval =  u_limit;
    } else if (val < l_limit) {
      outval = l_limit;
    }
    return outval;
  }

  public boolean liftAtMax() {
    return m_liftPos == Constants.Arm.LIFT_MAX;
  }

  public boolean liftAtMin() {
    return m_liftPos == Constants.Arm.LIFT_MIN;
  }

  public boolean tiltAtMax() {
    return m_tiltPos == Constants.Arm.TILT_MAX;
  }

  public boolean tiltAtMin() {
    return m_tiltPos == Constants.Arm.TILT_MIN;
  }

  public boolean armAtMax() {
    return tiltAtMax() && liftAtMax();
  }

  public boolean armAtMin() {
    return tiltAtMin() && liftAtMin();
  }

  @Override
  public void periodic() {
  }
}
