// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  /** Creates a new TurnToAngle. */
  public TurnToAngle(double targetAngleDegrees, Drivetrain drive) {
    super(
        // The controller that the command will use
        // new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),

        new PIDController(SmartDashboard.getNumber("P", 1.0), 
                          DriveConstants.kTurnI, 
                          SmartDashboard.getNumber("D", 0.0)),
        // This should return the measurement
        drive::getHeading,
        // This should return the setpoint (can also be a constant)
        () -> SmartDashboard.getNumber("Setpoint", 0),
        // targetAngleDegrees,
        // This uses the output
        output -> {
          // Use the output here
          drive.turn(output/10);
        },
        drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(DriveConstants.kTurnToleranceDeg,
                                DriveConstants.kTurnRateToleranceDegPerS);

    
    // System.out.println("Setpoint " + targetAngleDegrees);
  }

  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
    SmartDashboard.putNumber("Setpoint ", getController().getSetpoint());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
