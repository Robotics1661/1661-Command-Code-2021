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

public class VerticalAgitatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  WPI_TalonFX verticalAgitatorRight = new WPI_TalonFX(Constants.vertical_agitator_right);
  WPI_TalonFX verticalAgitatorLeft = new WPI_TalonFX(Constants.vertical_agitator_left);
  WPI_TalonFX verticalAgitatorFront = new WPI_TalonFX(Constants.vertical_agitator_front);

  public VerticalAgitatorSubsystem() {

    verticalAgitatorRight.configFactoryDefault();
    verticalAgitatorLeft.configFactoryDefault();
    verticalAgitatorFront.configFactoryDefault();

    verticalAgitatorRight.setSensorPhase(false);
    verticalAgitatorLeft.setSensorPhase(false);
    verticalAgitatorFront.setSensorPhase(false);
        
    verticalAgitatorRight.setInverted(false);
    verticalAgitatorLeft.setInverted(false);
    verticalAgitatorFront.setInverted(false);
  
    
    SmartDashboard.putNumber("vaEnc", 0.00);
    SmartDashboard.putBoolean("vaEncReset", false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (SmartDashboard.getBoolean("vaEncReset", false)) {
			verticalAgitatorRight.setSelectedSensorPosition(0);
			SmartDashboard.putBoolean("vaEncReset", false);
		}

    SmartDashboard.putNumber("vaEnc", verticalAgitatorRight.getSelectedSensorPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void spinUp() {
    verticalAgitatorRight.set(ControlMode.PercentOutput, .90);
    verticalAgitatorLeft.set(ControlMode.PercentOutput, -.90);
    verticalAgitatorFront.set(ControlMode.PercentOutput, .90);
}

public void spinVariable(double v) {
  verticalAgitatorRight.set(ControlMode.PercentOutput, -v);
  verticalAgitatorLeft.set(ControlMode.PercentOutput, v);
  verticalAgitatorFront.set(ControlMode.PercentOutput, -v);
}

public void spinDown() {
    verticalAgitatorRight.set(ControlMode.PercentOutput, -.5);
    verticalAgitatorLeft.set(ControlMode.PercentOutput, .5);
    verticalAgitatorFront.set(ControlMode.PercentOutput, -.5);
}

public void stop() {
    verticalAgitatorRight.set(ControlMode.PercentOutput, 0);
    verticalAgitatorLeft.set(ControlMode.PercentOutput, 0);
    verticalAgitatorFront.set(ControlMode.PercentOutput, 0);
}

}
