// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnDegreesPID extends PIDCommand {
  /** Creates a new TurnToAngle. */
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("Shuffleboard/Drivetrain");

  public TurnDegreesPID(double targetAngleDegrees, Drivetrain drive) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.kTurnP, 
                          DriveConstants.kTurnI, 
                          DriveConstants.kTurnD),

        // This should return the measurement
        drive::getHeading,

        // This should return the setpoint (can also be a constant)
        targetAngleDegrees,

        // This uses the output
        output -> {
          // Use the output here
          drive.turn(output/10);
        },
        // Use addRequirements() here to declare subsystem dependencies.
        drive);
    
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(DriveConstants.kTurnToleranceDeg,
                                DriveConstants.kTurnRateToleranceDegPerS);
  }

  public void initialize() {
    super.initialize();
    // Override PID parameters from Shuffleboard
    getController().setSetpoint(table.getEntry("Heading Angle").getDouble(0.0));
    getController().setP(table.getEntry("P").getDouble(1.0));
    getController().setD(table.getEntry("D").getDouble(0.0));

    SmartDashboard.putNumber("Setpoint", getController().getSetpoint());
    SmartDashboard.putNumber("Got P", getController().getP());
    SmartDashboard.putNumber("Got D", getController().getD());
  }

  public void execute() {
    // TODO Auto-generated method stub
    super.execute(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
