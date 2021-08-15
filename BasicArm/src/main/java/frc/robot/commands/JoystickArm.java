// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickArm extends CommandBase {

  private final Arm m_arm;
  private final Joystick m_joystick;
  private long m_startTime = 0;

  /* Creates a new command which controls the arm via 
   * a Joystick.
   * 
   * @param arm Arm subsystem
   * @param joystick Joystick to be used to control the arm
   * 
   */
  public JoystickArm(Arm arm, Joystick joystick) {
    m_arm = arm;
    m_joystick = joystick;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(System.currentTimeMillis() - m_startTime > 500 ) {
      // System.out.println("Gripper Pos " + m_arm.getGripperFeedbackPos());
      m_startTime = System.currentTimeMillis();
    }

    if(m_joystick.getRawButton(Constants.Joystick.CIRCLE_BUTTON)) {
      m_arm.incrementLift(-Constants.Arm.SERVO_INCREMENT);
      System.out.println("Lift -" );
    }
    if(m_joystick.getRawButton(Constants.Joystick.TRIANGLE_BUTTON)) {
      m_arm.incrementLift(Constants.Arm.SERVO_INCREMENT);
      System.out.println("Lift +" );
    }
    if(m_joystick.getRawButton(Constants.Joystick.TOP_DIR)) {
      m_arm.incrementTilt(Constants.Arm.SERVO_INCREMENT);
      System.out.println("Tilt +" );
    }
    if(m_joystick.getRawButton(Constants.Joystick.LEFT_DIR)) {
      m_arm.incrementTilt(-Constants.Arm.SERVO_INCREMENT);
      System.out.println("Tilt -" );
    }
    if(m_joystick.getRawButton(Constants.Joystick.L2_BUTTON)) {
      m_arm.incrementGripper(Constants.Arm.SERVO_INCREMENT);
      System.out.println("Gripper Pos + " + m_arm.getGripperFeedbackPos());
    }
    if(m_joystick.getRawButton(Constants.Joystick.R2_BUTTON)) {
      m_arm.incrementGripper(-Constants.Arm.SERVO_INCREMENT);
      System.out.println("Gripper Pos - " + m_arm.getGripperFeedbackPos());
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
