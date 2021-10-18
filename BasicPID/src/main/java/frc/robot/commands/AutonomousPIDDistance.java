// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousPIDDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  
  public AutonomousPIDDistance(Drivetrain drivetrain) {
    drivetrain.arcadeDrive(0, 0);
    drivetrain.resetGyro();
    drivetrain.resetEncoders();
    // Drive in a square
    addCommands(
        new DriveDistanceProfiled(0.5, drivetrain),
        new TurnDegreesPID(90, drivetrain),
        new DriveDistanceProfiled(1.0, drivetrain),
        new TurnDegreesPID(180, drivetrain),
        new DriveDistanceProfiled(1.5, drivetrain),
        new TurnDegreesPID(270, drivetrain),
        new DriveDistanceProfiled(2.0, drivetrain),
        new TurnDegreesPID(0, drivetrain));
  }
}