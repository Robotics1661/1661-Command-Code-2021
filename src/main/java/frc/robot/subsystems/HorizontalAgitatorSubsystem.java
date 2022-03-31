// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HorizontalAgitatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    WPI_TalonFX horizontalAgitatorRight = new WPI_TalonFX(Constants.horizontal_agitator_right);
    WPI_TalonFX horizontalAgitatorLeft = new WPI_TalonFX(Constants.horizontal_agitator_left);

  public HorizontalAgitatorSubsystem() {

    horizontalAgitatorRight.configFactoryDefault();
    horizontalAgitatorLeft.configFactoryDefault();
    
    horizontalAgitatorRight.setSensorPhase(false);
    horizontalAgitatorLeft.setSensorPhase(false);
    
    horizontalAgitatorRight.setInverted(false);
    horizontalAgitatorLeft.setInverted(false);


    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void moveToVA() {
      horizontalAgitatorLeft.set(ControlMode.PercentOutput, -.4);
      horizontalAgitatorRight.set(ControlMode.PercentOutput, .4);
  }

  public void moveFromVA() {
    horizontalAgitatorLeft.set(ControlMode.PercentOutput, .4);
    horizontalAgitatorRight.set(ControlMode.PercentOutput, -.4);
  }

  public void stop() {
    horizontalAgitatorLeft.set(ControlMode.PercentOutput, 0);
    horizontalAgitatorRight.set(ControlMode.PercentOutput, 0);
}

}
