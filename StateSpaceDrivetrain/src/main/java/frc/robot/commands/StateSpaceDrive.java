// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTableEntry;

public class StateSpaceDrive extends CommandBase {
  /** Creates a new StateSpaceDrive. */
  private final Drivetrain m_drive;
  private final double m_speed;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(Constants.DriveConstants.maxVelocityPerSecond,
                                       Constants.DriveConstants.maxAccelPerSecond); 
  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  // Get data from Shuffleboard
  // NetworkTableInstance inst = NetworkTableInstance.getDefault();
  // NetworkTable table = inst.getTable("Shuffleboard/Drivetrain");
  private ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain");
  private NetworkTableEntry m_leftVoltage = 
    driveTab.add("Left Voltage", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(3, 0)
      .getEntry();
  private NetworkTableEntry m_kalmanGain = 
    driveTab.add("Kalman Gain", 0)
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(5, 0)
      .getEntry();    

  /**
   * Constructs a StateSpaceDrive command
   *
   * @param speed Speed at which the robot should travel.
   * @param drive The Drivetrain
   */
  public StateSpaceDrive(double speed, Drivetrain drive) {
    m_drive = drive;
    m_speed = speed;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_drive.m_loop.reset(Matrix.mat(Nat.N2(), Nat.N1()).fill(0,0));

    // Report starting voltage to Shuffleboard
    m_leftVoltage.setDouble(m_drive.m_loop.getU(0));

    // double nextLeftVoltage = m_drive.m_loop.getU(0);
    // double nextRightVoltage = m_drive.m_loop.getU(1);
    // SmartDashboard.putNumber("Left Voltage", nextLeftVoltage);
    // SmartDashboard.putNumber("Right Voltage", nextRightVoltage);
    
    // System.out.println("Running StateSpace Command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets the target speed of our drivetrain. This is similar to setting the setpoint of a
    // PID controller.
    // We just pressed the trigger, so let's set our next reference
    // goal = new TrapezoidProfile.State(kHighGoalPosition, 0.0);
    m_drive.m_loop.setNextR(VecBuilder.fill(m_speed, m_speed));

    // Correct our Kalman filter's state vector estimate with encoder data.
    // Get the current rate of the encoder. Units are distance per second as 
    // scaled by the value from setDistancePerPulse().
    double leftEncoderRate = m_drive.getLeftEncoderRate();
    double rightEncoderRate = m_drive.getLeftEncoderRate();
    m_drive.m_loop.correct(VecBuilder.fill(leftEncoderRate,rightEncoderRate));

    // SmartDashboard.putNumber("Left Encoder Rate", leftEncoderRate);
    // SmartDashboard.putNumber("Right Encoder Rate", rightEncoderRate);

    // Update our LQR to generate new voltage commands and use the voltages to 
    // predict the next state with our Kalman filter.
    m_drive.m_loop.predict(0.020);

    double kalmanGain = m_drive.m_loop.getController().getK().get(0, 0);
    m_kalmanGain.setDouble(kalmanGain);
    // SmartDashboard.putNumber("Kalman Gain", kalmanGain);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextLeftVoltage = m_drive.m_loop.getU(0);
    double nextRightVoltage = m_drive.m_loop.getU(1);
    m_drive.setLeftVoltage(nextLeftVoltage);
    m_drive.setRightVoltage(-nextRightVoltage);

    m_leftVoltage.setDouble(nextLeftVoltage);

    // SmartDashboard.putNumber("Left Voltage", nextLeftVoltage);
    // SmartDashboard.putNumber("Right Voltage", nextRightVoltage);
    // System.out.println("executing");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Put the setpoint to zero
    m_drive.m_loop.setNextR(VecBuilder.fill(0.0, 0.0));

    // Report last voltage
    double nextLeftVoltage = m_drive.m_loop.getU(0);
    m_leftVoltage.setDouble(nextLeftVoltage);

    // double nextLeftVoltage = m_drive.m_loop.getU(0);
    // double nextRightVoltage = m_drive.m_loop.getU(1);
    // SmartDashboard.putNumber("Left Voltage", nextLeftVoltage);
    // SmartDashboard.putNumber("Right Voltage", nextRightVoltage);

    // Report last speed
    // double leftEncoderRate = m_drive.getLeftEncoderRate();
    // double rightEncoderRate = m_drive.getLeftEncoderRate();
    // SmartDashboard.putNumber("Left Encoder Rate", leftEncoderRate);
    // SmartDashboard.putNumber("Right Encoder Rate", rightEncoderRate);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
