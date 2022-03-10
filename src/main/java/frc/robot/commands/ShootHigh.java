// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HorizontalAgitatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VerticalAgitatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/** An example command that uses an example subsystem. */
public class ShootHigh extends ParallelCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final HorizontalAgitatorSubsystem m_haSubsystem;
  private final VerticalAgitatorSubsystem m_vaSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  double startTime;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootHigh(HorizontalAgitatorSubsystem ha, VerticalAgitatorSubsystem va, ShooterSubsystem shooter) {
    m_haSubsystem = ha;
    m_vaSubsystem = va;
    m_shooterSubsystem = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ha, va, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vaSubsystem.spinUp();
    m_shooterSubsystem.shoot();
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = Timer.getFPGATimestamp();
    if (time - startTime > 3) {
        m_haSubsystem.moveToVA();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_haSubsystem.stop();
      m_vaSubsystem.stop();
      m_shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
