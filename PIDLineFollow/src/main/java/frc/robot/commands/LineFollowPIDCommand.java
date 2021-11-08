/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;

import frc.robot.subsystems.Vision;

public class LineFollowPIDCommand extends PIDCommand {
    private static Drivetrain m_drive;
    private final Vision visionSubsystem;

    public LineFollowPIDCommand(Drivetrain drive) {
        super(
            new PIDController(DriveConstants.kPDriveVel,
                              DriveConstants.kIDriveVel,
                              DriveConstants.kDDriveVel), 
            RobotContainer.m_vision::getCenterX, 
            VisionConstants.SETPOINT,
            output -> {
                // Use the output here
                drive.steer(-output);
              },
            RobotContainer.m_drivetrain
        );
        m_drive = RobotContainer.m_drivetrain;
        visionSubsystem = RobotContainer.m_vision;
    }

    public void initialize() {
        super.initialize();
        // Make sure everything starts from zero
        m_drive.arcadeDrive(0, 0);
    }

    public void execute() {
        super.execute();
        SmartDashboard.putNumber("Error", getController().getPositionError());
        SmartDashboard.putBoolean("Finished", getController().atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return visionSubsystem.getRectArea() < VisionConstants.END_OF_LINE;
    }
}
