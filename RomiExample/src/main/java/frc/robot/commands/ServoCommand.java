// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.RomiServo;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ServoCommand extends CommandBase {

  private final RomiServo m_servo;
  private final Joystick m_joystick;

  /* Creates a new command which controls the romi_servo via 
   * a Joystick.
   * 
   * @param romi_servo Servo subsystem
   * @param joystick Joystick to be used to control the romi_servo
   * 
   */
  public ServoCommand(RomiServo romi_servo, Joystick joystick) {
    m_servo = romi_servo;
    m_joystick = joystick;

    addRequirements(romi_servo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_joystick.getRawButton(Constants.Joystick.TOPLEFT)) {
      m_servo.incrementLift(-Constants.Servo.SERVO_INCREMENT);
      System.out.println("Lift -" );
    }
    if(m_joystick.getRawButton(Constants.Joystick.TOPRIGHT)) {
      m_servo.incrementLift(Constants.Servo.SERVO_INCREMENT);
      System.out.println("Lift +" );
    }
    if(m_joystick.getRawButton(Constants.Joystick.BOTTOMLEFT)) {
      m_servo.incrementTilt(Constants.Servo.SERVO_INCREMENT);
      System.out.println("Tilt +" );
    }
    if(m_joystick.getRawButton(Constants.Joystick.BOTTOMRIGHT)) {
      m_servo.incrementTilt(-Constants.Servo.SERVO_INCREMENT);
      System.out.println("Tilt -" );
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
