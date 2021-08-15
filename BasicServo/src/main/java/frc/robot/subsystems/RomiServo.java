// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiServo extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Servo m_servo1 = new Servo(Constants.Servo.SERVO1_PORT);
  private final Servo m_servo2 = new Servo(Constants.Servo.SERVO2_PORT);
  private double m_servo1Pos;
  private double m_servo2Pos;

  // Creates a new Servo subsystem
  public RomiServo() {
    reset();
  }

  // Reset position to resting state
  public void reset() {
    m_servo1Pos = 0.5;
    m_servo2Pos = 0.5;
    
    m_servo1.set(m_servo1Pos);
    m_servo2.set(m_servo2Pos);
  }

  /** 
   * Increment servo1 motor position
   * 
   * @param delta Amount to change motor position
   */
  public void incrementServo1(double delta) {
    m_servo2Pos = saturateLimit(m_servo2Pos + delta, Constants.Servo.MIN_RANGE, Constants.Servo.MAX_RANGE);
    m_servo2.set(m_servo2Pos);
  }

  /**
   * Increment servo2 motor position
   * 
   * @param delta Amount to change motor position
   */
  public void incrementServo2(double delta) {
    m_servo1Pos = saturateLimit(m_servo1Pos + delta,  Constants.Servo.MIN_RANGE,  Constants.Servo.MAX_RANGE); 
    m_servo1.set(m_servo1Pos);
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
