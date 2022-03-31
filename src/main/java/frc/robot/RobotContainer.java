// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FindTarget;
import frc.robot.commands.InstantHighShot;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.RunHorizontalAgitator;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.ShootLow;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.EjectBall;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HorizontalAgitatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;
import frc.robot.subsystems.VerticalAgitatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  private final UltrasonicSubsystem m_ultrasonicSubsystem = new UltrasonicSubsystem();
  private final RunHorizontalAgitator m_runHorizontalAgitator = new RunHorizontalAgitator(m_hAgitatorSubsystem);

  private final AutoCommand m_autoCommand = new AutoCommand(m_driveSubsystem, m_intakeSubsystem, m_hAgitatorSubsystem, m_vAgitatorSubsystem, m_shooterSubsystem);
  private final IntakeBall m_intakeBall = new IntakeBall(m_intakeSubsystem);
  private final EjectBall m_ejectBall = new EjectBall(m_hAgitatorSubsystem, m_intakeSubsystem);
  private final ShootHigh m_shootHigh = new ShootHigh(m_hAgitatorSubsystem, m_vAgitatorSubsystem, m_shooterSubsystem);
  private final ShootLow m_shootLow = new ShootLow(m_hAgitatorSubsystem, m_vAgitatorSubsystem, m_shooterSubsystem);
  private final FindTarget m_findTarget = new FindTarget(m_driveSubsystem, m_visionSubsystem);
  private final InstantHighShot m_instantHighShot = new InstantHighShot(m_hAgitatorSubsystem, m_vAgitatorSubsystem, m_shooterSubsystem);

  XboxController m_driveController = new XboxController(0);

  OI oi = new OI();

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

    // m_hAgitatorSubsystem.setDefaultCommand(
    //   new RunCommand(
    //     () ->
    //       m_hAgitatorSubsystem.moveToVA(),
    //       m_hAgitatorSubsystem
    //   )
    // );  
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // Intake Ball
    oi.m_bButton.whileActiveContinuous(m_intakeBall);

    // Eject Ball
    oi.m_leftBumper.whileActiveContinuous(m_ejectBall);

    // Shoot Ball -> Upper Hub
    oi.m_rightTrigger.whileActiveContinuous(m_shootHigh); // Ramp up
    oi.m_aButton.whileActiveContinuous(m_instantHighShot); // No ramp up

    // Shoot Ball -> Lower Hub
    oi.m_leftTrigger.whileActiveContinuous(m_shootLow);

    // Find Target
    oi.m_xButton.whileActiveContinuous(m_findTarget);
    oi.d_aButton.whileActiveContinuous(m_findTarget);

    // Run HA towards VA
    oi.m_yButton.whileActiveContinuous(m_runHorizontalAgitator);


    
  }

  public XboxController getDriveBox() {
    return m_driveController;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Trajectory customTrajectory) {
    // An ExampleCommand will run in autonomous

    // Create a voltage constraint to ensure we don't accelerate too fast
  //   var autoVoltageConstraint =
  //       new DifferentialDriveVoltageConstraint(
  //           new SimpleMotorFeedforward(
  //               Constants.ksVolts,
  //               Constants.kvVoltSecondsPerMeter,
  //               Constants.kaVoltSecondsSquaredPerMeter),
  //           Constants.kDriveKinematics,
  //           1);
            
  //   // Create config for trajectory
  //   TrajectoryConfig config =
  //       new TrajectoryConfig(
  //               Constants.kMaxSpeedMetersPerSecond,
  //               Constants.kMaxAccelerationMetersPerSecondSquared)
  //           // Add kinematics to ensure max speed is actually obeyed
  //           .setKinematics(Constants.kDriveKinematics)
  //           // Apply the voltage constraint
  //           .addConstraint(autoVoltageConstraint);

  // //  An example trajectory to follow.  All units in meters.
  //  Trajectory exampleTrajectory =
  //  TrajectoryGenerator.generateTrajectory(
  //      // Start at the origin facing the +X direction
  //      new Pose2d(0, 0, new Rotation2d(0)),
  //      // Pass through these two interior waypoints, making an 's' curve path
  //      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  //      // End 3 meters straight ahead of where we started, facing forward
  //      new Pose2d(3, 0, new Rotation2d(0)),
  //      // Pass config
  //      config);

  //   // Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(initial, interiorWaypoints, end, config)
    

  //   // Custom trajectory
  //   // Trajectory exampleTrajectory = customTrajectory;
  //   // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(initial, interiorWaypoints, end, config)

  //     RamseteCommand ramseteCommand =
  //     new RamseteCommand(
  //         exampleTrajectory,
  //         m_driveSubsystem::getPose,
  //         new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
  //         new SimpleMotorFeedforward(
  //             Constants.ksVolts,
  //             Constants.kvVoltSecondsPerMeter,
  //             Constants.kaVoltSecondsSquaredPerMeter),
  //         Constants.kDriveKinematics,
  //         m_driveSubsystem::getWheelSpeeds,
  //         new PIDController(Constants.kPDriveVel, 0, 0),
  //         new PIDController(Constants.kPDriveVel, 0, 0),
  //         // RamseteCommand passes volts to the callback
  //         m_driveSubsystem::tankDriveVolts,
  //         m_driveSubsystem);

  //   // Reset odometry to the starting pose of the trajectory.
  //   m_driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

  //   // Run path following command, then stop at the end.
  //   return ramseteCommand.andThen(() -> m_driveSubsystem.tankDriveVolts(0, 0));

    return m_autoCommand;
  }

}
