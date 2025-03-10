// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.IO.JoystickIO;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.JoystickArm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

// New components added to this project.
import frc.robot.subsystems.Arm;
import frc.robot.commands.PositionArm;
import frc.robot.commands.PositionLift;
import frc.robot.commands.PositionTilt;
import frc.robot.commands.StopMotors;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
  public final Arm m_arm = new Arm();

  // Assumes a gamepad plugged into channnel 0
  private final XboxController m_joystick = new XboxController(0);
  private final JoystickIO m_joystickIO = new JoystickIO(m_joystick);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());
    // m_arm.setDefaultCommand(setTiltLevel());

    // Example of how to use the onboard IO
    Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .whenActive(new PrintCommand("Button A Pressed"))
        .whenInactive(new PrintCommand("Button A Released"));

    // Example of how to use the external IO
    Button extIO0Button = new Button(m_onboardIO::getExt0Pressed);
    extIO0Button
        .whenActive(new StopMotors(m_drivetrain))
        .whenInactive(new PrintCommand("Bumper Released"));    

    // Setup bindings to control the arm
    configureArmBindings();
    
    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auto Move Arm UP", new PositionArm(m_arm, 1));
    m_chooser.addOption("Auto Move Arm DOWN", new PositionArm(m_arm, -1));
    SmartDashboard.putData(m_chooser);
  }

  public void configureArmBindings() {
    m_arm.setDefaultCommand( new JoystickArm(m_arm, m_joystickIO));

    // Move Arm full UP
    m_joystickIO.moveArmFullUp().whenPressed(new PositionArm(m_arm, 1));
    
    // Move Arm full DOWN
    m_joystickIO.moveArmFullDown().whenPressed(new PositionArm(m_arm, -1));
      
    // Add commands to Shuffleboard
    SmartDashboard.putData("Arm UP", new PositionArm(m_arm, 1));  
    SmartDashboard.putData("Arm DOWN", new PositionArm(m_arm, -1));   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_joystick.getRawAxis(1), () -> m_joystick.getRawAxis(0));
  }

  /**
   * Set the default command for the Arm
   * Set the tilt level
   * 
   */
  public Command setTiltLevel() {
    return new PositionArm(m_arm, 1);
  }
}
