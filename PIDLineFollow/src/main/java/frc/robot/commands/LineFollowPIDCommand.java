/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import frc.robot.subsystems.Vision;

public class LineFollowPIDCommand extends PIDCommand {
    private final Drivetrain chassisSubsystem;
    private final Vision visionSubsystem;

    public LineFollowPIDCommand(Drivetrain drive) {
        super(
            new PIDController(Constants.Drive.KP, 0, 0), 
            RobotContainer.m_vision::getCenterX, 
            Constants.Vision.SETPOINT,
            output -> {
                // Use the output here
                drive.steer(-output);
              },
            RobotContainer.m_drivetrain
        );
        chassisSubsystem = RobotContainer.m_drivetrain;
        visionSubsystem = RobotContainer.m_vision;
    }

    public void initialize() {
        super.initialize();
        // Make sure everything starts from zero
        RobotContainer.m_drivetrain.arcadeDrive(0, 0);
        RobotContainer.m_drivetrain.resetOutput();
    }

    @Override
    public void end(boolean interrupted) {
        chassisSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return visionSubsystem.getRectArea() < Constants.Vision.END_OF_LINE;
    }
}
