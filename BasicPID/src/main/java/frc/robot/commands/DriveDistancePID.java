// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistancePID extends PIDCommand {
  /** Creates a new DriveDistancePID. */
  private static Drivetrain m_drive;
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("Shuffleboard/Drivetrain");
  
  public DriveDistancePID(double targetDistance, Drivetrain drive) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.kPDriveVel,
                          DriveConstants.kIDriveVel,
                          DriveConstants.kDDriveVel),
        // This should return the measurement
        drive::getAverageDistanceMeters,
        // This should return the setpoint (can also be a constant)
        targetDistance,
        // This uses the output
        output -> {
          // Use the output here
          drive.steer(output);
        },
        // Use addRequirements() here to declare subsystem dependencies.
        drive);
    
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(DriveConstants.kDistanceToleranceMeters,
                                DriveConstants.kVelocityToleranceMetersPerS);

    m_drive = drive;
  }

  public void initialize() {
    super.initialize();
    // Override PID parameters from Shuffleboard
    getController().setSetpoint(table.getEntry("Distance").getDouble(0.0));
    getController().setP(table.getEntry("distanceP").getDouble(1.0));
    getController().setD(table.getEntry("distanceD").getDouble(0.0));
  }
  

  public void execute() {
    super.execute();
    SmartDashboard.putNumber("Error", getController().getPositionError());
    SmartDashboard.putBoolean("Finished", getController().atSetpoint());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  public void resetOdometry() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetGyro();
    m_drive.resetEncoders();
  }
}
