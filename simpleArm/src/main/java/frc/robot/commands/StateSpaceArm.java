// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class StateSpaceArm extends CommandBase {
  /** Creates a new StateSpaceArm. */
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(
          Units.degreesToRadians(45),
          Units.degreesToRadians(90)); // Max arm speed and acceleration.
  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();


  private final Arm m_arm;
  private double m_direction = 0;

  public StateSpaceArm(double direction, Arm arm) {
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
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
