// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PositionArm extends CommandBase {
  private final Arm m_arm;
  private double m_direction = 0;
  private double m_currentTilt = 0;
  private double m_currentLift = 0;

  /* Creates a new PositionArm. 
   *  
   * @param arm Arm subsystem
   * @param joystick Joystick to be used to control the arm
   * 
  */
  public PositionArm(Arm arm, double direction) {
    m_arm = arm;
    m_direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the current Lift and Tilt positions
    m_currentLift = m_arm.getLiftPos();
    m_currentTilt = m_arm.getTiltPos();
    // System.out.println("Pos Lift " + m_currentLift);
    // System.out.println("Pos Tilt " + m_currentTilt);

    // Move the arm up until is reaches its max position
    if(m_direction == 1) {
      System.out.println("Move UP");
      m_arm.incrementLift(Constants.Arm.SERVO_INCREMENT);  
      // TILT always at max   
      m_arm.incrementTilt(Constants.Arm.SERVO_INCREMENT);    
    }

    // Move the arm down until is reaches its max position
    if(m_direction == 0) {
      System.out.println("Move DOWN");
      m_arm.incrementLift(-Constants.Arm.SERVO_INCREMENT);
      // TILT always at max
      m_arm.incrementTilt(Constants.Arm.SERVO_INCREMENT);       
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_direction == 1) {
      if (m_arm.liftAtMax() && m_arm.tiltAtMax()) {
        System.out.println("FINISHED UP");
        return true;
      }  
    } else {
      if (m_direction == 0) {
        if (m_arm.liftAtMin() && m_arm.tiltAtMax()) {
          System.out.println("FINISHED DOWN");
          return true;
        } 
      }      
    }  
    return false;
  }
}
