// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiServo extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Servo m_lift = new Servo(Constants.Servo.LIFT_PORT);
  private final Servo m_tilt = new Servo(Constants.Servo.TILT_PORT);
  private double m_liftPos;
  private double m_tiltPos;

  // Creates a new Servo subsystem
  public RomiServo() {
    reset();
  }

  // Reset position to resting state
  public void reset() {
    m_liftPos = 0.5;
    m_tiltPos = 0.5;
    
    m_lift.set(m_liftPos);
    m_tilt.set(m_tiltPos);
  }

  /** 
   * Increment tilt motor position
   * 
   * @param delta Amount to change motor position
   */
  public void incrementTilt(double delta) {
    m_tiltPos = saturateLimit(m_tiltPos + delta, Constants.Servo.TILT_MIN, Constants.Servo.TILT_MAX);
    m_tilt.set(m_tiltPos);
  }

  /**
   * Increment lift motor position
   * 
   * @param delta Amount to change motor position
   */
  public void incrementLift(double delta) {
    m_liftPos = saturateLimit(m_liftPos + delta,  Constants.Servo.LIFT_MIN,  Constants.Servo.LIFT_MAX); 
    m_lift.set(m_liftPos);
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

  @Override
  public void periodic() {
  }
}
