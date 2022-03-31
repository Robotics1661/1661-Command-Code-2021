// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  TalonFX shooter = new TalonFX(Constants.shooter_motor);

  public ShooterSubsystem() {

    shooter.configFactoryDefault();
    
    shooter.setSensorPhase(false);
    
    shooter.setInverted(false);

    shooter.config_kP(0, 0.3, Constants.kTimeoutMs);

    SmartDashboard.putNumber("shooterEnc", 0.00);
    SmartDashboard.putBoolean("shooterEncReset", false);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (SmartDashboard.getBoolean("shooterEncReset", false)) {
			shooter.setSelectedSensorPosition(0);
			SmartDashboard.putBoolean("shooterEncReset", false);
		}

    SmartDashboard.putNumber("shooterEnc", shooter.getSelectedSensorPosition());
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

    public void shoot() {
        shooter.set(ControlMode.PercentOutput, .99);
    }

    public void shootVariable(double v) {
        shooter.set(ControlMode.PercentOutput, v);
    }

    public void shootSmall() {
        shooter.set(ControlMode.PercentOutput, .45);
    }

    public void stop() {
        shooter.set(ControlMode.PercentOutput, 0);
    }

}
