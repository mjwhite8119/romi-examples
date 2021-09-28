// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnDegreesProfiled extends ProfiledPIDCommand {
  /** Creates a new TurnToAngleProfiled. */
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("Shuffleboard/Drivetrain");

  public TurnDegreesProfiled(double targetAngleDegrees, Drivetrain drive) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(DriveConstants.kTurnP, 
                                  DriveConstants.kTurnI, 
                                  DriveConstants.kTurnD,
          // The motion profile constraints
          new TrapezoidProfile.Constraints(Constants.DriveConstants.kMaxVelocityMeters,
                                            Constants.DriveConstants.kMaxAcceMeters)),

        // This should return the measurement
        drive::getHeading,

        // This should return the goal/setpoint (can also be a constant)
        () -> new TrapezoidProfile.State(targetAngleDegrees, 0),

        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drive.turn(output);
        },

        drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.getController().enableContinuousInput(-180, 180);
    getController().setTolerance(DriveConstants.kTurnToleranceDeg,
                                DriveConstants.kTurnRateToleranceDegPerS);

  }

  public void initialize() {
    super.initialize();
    // Override PID parameters from Shuffleboard
    getController().setGoal(table.getEntry("Heading Angle Degrees").getDouble(0.0));
    getController().setP(table.getEntry("anglekP").getDouble(1.0));
    getController().setD(table.getEntry("anglekD").getDouble(0.0));
  }

  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
    SmartDashboard.putNumber("goal (deg.)", getController().getGoal().position);
    SmartDashboard.putNumber("setpoint (deg.)", getController().getSetpoint().position);
    SmartDashboard.putNumber("Pos. Error (deg.)", getController().getPositionError());
    SmartDashboard.putNumber("Vel. Error (deg.)", getController().getVelocityError());
    SmartDashboard.putBoolean("atSetpoint (deg.)", getController().atSetpoint());
    SmartDashboard.putBoolean("atGoal (deg.)", getController().atGoal());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
