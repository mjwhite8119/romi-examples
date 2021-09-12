// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PositionLift extends CommandBase {
    private final Arm m_arm;
    private double m_direction = 0;

    /*
     * Creates a new PositionLift command.
     * 
     * @param arm Arm subsystem
     * 
     * @param direction of travel for the arm
     * 
     */
    public PositionLift(Arm arm, double direction) {
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

    // Move the Lift up until is reaches its max position
    if(m_direction == 1) {
      System.out.println("Lift UP " + m_arm.getLiftPos());
      m_arm.incrementLift(Constants.Arm.SERVO_INCREMENT);          
    }

    // Move the Lift down until is reaches its max position
    if(m_direction == 0) {
      System.out.println("Lift DOWN " + m_arm.getLiftPos());
      m_arm.incrementLift(-Constants.Arm.SERVO_INCREMENT);       
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_direction == 1) {
      if (m_arm.liftAtMax()) {
        System.out.println("FINISHED UP Lift=" + m_arm.getLiftPos());
        return true;
      }  
    } else {
      if (m_direction == 0) {
        if (m_arm.liftAtMin()) {
          System.out.println("FINISHED DOWN Lift=" + m_arm.getLiftPos());
          return true;
        } 
      }      
    }  
    return false;
  }
}

