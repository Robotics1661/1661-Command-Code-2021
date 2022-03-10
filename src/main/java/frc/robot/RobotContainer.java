// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FindTarget;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.ShootLow;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.EjectBall;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HorizontalAgitatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VerticalAgitatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final HorizontalAgitatorSubsystem m_hAgitatorSubsystem = new HorizontalAgitatorSubsystem();
  private final VerticalAgitatorSubsystem m_vAgitatorSubsystem = new VerticalAgitatorSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  private final AutoCommand m_autoCommand = new AutoCommand(m_driveSubsystem, m_intakeSubsystem, m_hAgitatorSubsystem, m_vAgitatorSubsystem, m_shooterSubsystem);
  private final IntakeBall m_intakeBall = new IntakeBall(m_intakeSubsystem);
  private final EjectBall m_ejectBall = new EjectBall(m_hAgitatorSubsystem, m_intakeSubsystem);
  private final ShootHigh m_shootHigh = new ShootHigh(m_hAgitatorSubsystem, m_vAgitatorSubsystem, m_shooterSubsystem);
  private final ShootLow m_shootLow = new ShootLow(m_hAgitatorSubsystem, m_vAgitatorSubsystem, m_shooterSubsystem);
  private final FindTarget m_findTarget = new FindTarget(m_driveSubsystem, m_visionSubsystem);

  XboxController m_driveController = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(
      new RunCommand(
        () ->
          m_driveSubsystem.executeTank(-m_driveController.getRawAxis(Constants.left_y_axis), m_driveController.getRawAxis(Constants.right_x_axis)),
          m_driveSubsystem
      )
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton m_aButton = new JoystickButton(m_driveController, Constants.a_button);
    JoystickButton m_bButton = new JoystickButton(m_driveController, Constants.b_button);
    JoystickButton m_xButton = new JoystickButton(m_driveController, Constants.x_button);
    JoystickButton m_yButton = new JoystickButton(m_driveController, Constants.y_button);

    JoystickButton m_leftBumper = new JoystickButton(m_driveController, Constants.left_bumper);

    // Intake Ball
    m_bButton.whileActiveContinuous(m_intakeBall);

    // Eject Ball
    m_leftBumper.whileActiveContinuous(m_ejectBall);

    // Shoot Ball -> Upper Hub
    m_aButton.whileActiveContinuous(m_shootHigh);

    // Shoot Ball -> Lower Hub
    m_yButton.whileActiveContinuous(m_shootLow);

    // Find Target
    m_xButton.whileActiveContinuous(m_findTarget);
    
  }

  public XboxController getDriveBox() {
    return m_driveController;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    return m_autoCommand;
  }

}
