// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistanceProfiled extends ProfiledPIDCommand {
  /** Creates a new DriveDistanceProfiled. */
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("Shuffleboard/Drivetrain");
  
  public DriveDistanceProfiled(double targetDistance, Drivetrain drive) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            DriveConstants.kDistanceP, 
            DriveConstants.kDistanceI, 
            DriveConstants.kDistanceD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0.15, 2.0)),
        // This should return the measurement
        drive::getAverageDistanceInch,
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(targetDistance,0),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drive.steer(output/10);
        },
        // Use addRequirements() here to declare subsystem dependencies.
        drive);
    
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(DriveConstants.kDistanceToleranceInch,
                                DriveConstants.kVelocityToleranceInchPerS);
  }

  public void initialize() {
    super.initialize();
    // Override PID parameters from Shuffleboard
    getController().setGoal(table.getEntry("Distance").getDouble(0.0));
    getController().setP(table.getEntry("distanceP").getDouble(1.0));
    getController().setD(table.getEntry("distanceD").getDouble(0.0));
  }

  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
    SmartDashboard.putNumber("Error", getController().getPositionError());
    SmartDashboard.putBoolean("Finished", getController().atSetpoint());
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
