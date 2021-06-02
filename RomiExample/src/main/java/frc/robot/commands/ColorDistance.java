// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.sensors.ColorSensor;
import frc.robot.utilities.MatchedColor;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ColorDistance extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_distance;
  private double m_speed;
  private double m_last_speed;
  private final ColorSensor m_colorSensor;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public ColorDistance(double speed, double inches, ColorSensor colorSensor, Drivetrain drive) {
    m_distance = inches;
    m_speed = speed;
    m_last_speed = speed;
    m_drive = drive;
    m_colorSensor = colorSensor;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    MatchedColor m_match = m_colorSensor.getMatchedColor();
    
    switch (m_match) {
      case RED:
        System.out.print("Red");
        m_speed = 0.0;
        break;

      case YELLOW:
        System.out.print("Yellow");
        m_speed = 0.6;
        break;

      case BLUE:
        System.out.print("Blue");
        m_speed = 0.8; 
        break;

      default:  
        System.out.print("Floor");  
        // m_speed = m_last_speed;
        break;
    }
    System.out.print(m_speed); 
    m_last_speed = m_speed;
    m_drive.arcadeDrive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
}
