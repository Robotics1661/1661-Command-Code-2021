// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  TalonSRX intake = new TalonSRX(Constants.intake_motor);

  public IntakeSubsystem() {

    intake.configFactoryDefault();
    
    intake.setSensorPhase(false);
    
    intake.setInverted(false);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

    /**
     *  <p>Intake the ball by spinning the intake.</p>
    */
    public void spin() {
        intake.set(ControlMode.PercentOutput, -.5);
    }

    /**
     *  <p>Discharge the ball by spinning the intake in reverse.</p>
    */
    public void spinReverse() {
        intake.set(ControlMode.PercentOutput, .5);
    }

    public void stop() {
        intake.set(ControlMode.PercentOutput, 0);
    }

}
