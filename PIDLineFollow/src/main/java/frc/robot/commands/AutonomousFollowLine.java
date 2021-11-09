// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousFollowLine extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Line Following command. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousFollowLine(Drivetrain drivetrain) {
    addCommands(
        new LineFollowPIDCommand(drivetrain),
        new TurnToAngleProfiled(180, drivetrain),
        new LineFollowPIDCommand(drivetrain));
  }
}
