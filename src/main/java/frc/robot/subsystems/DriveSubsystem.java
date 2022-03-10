// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    WPI_TalonFX fLeft = new WPI_TalonFX(Constants.front_left_drive);
    WPI_TalonFX fRight = new WPI_TalonFX(Constants.front_right_drive);
    WPI_TalonFX bLeft = new WPI_TalonFX(Constants.back_left_drive);
    WPI_TalonFX bRight = new WPI_TalonFX(Constants.back_right_drive);
    
    MotorControllerGroup left = new MotorControllerGroup(fLeft, bLeft);
    MotorControllerGroup right = new MotorControllerGroup(fRight, bRight);
    
    DifferentialDrive tank = new DifferentialDrive(left, right);

    private static final int kPIDLoopIdx = 0;
    private static final int kSlotIdx = 0;

    double x, y, throttle, turn, speedL, speedR, t_left, t_right;

  public DriveSubsystem() {

        fLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.peak_current, Constants.continuous_current, 0.5));
        fRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.peak_current, Constants.continuous_current, 0.5));
        bLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.peak_current, Constants.continuous_current, 0.5));
        bRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.peak_current, Constants.continuous_current, 0.5));
        
        fLeft.configOpenloopRamp(Constants.open_ramp);
        fRight.configOpenloopRamp(Constants.open_ramp);
        bLeft.configOpenloopRamp(Constants.open_ramp);
        bRight.configOpenloopRamp(Constants.open_ramp);

        fLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, Constants.kTimeoutMs);
        fRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, Constants.kTimeoutMs);
        bLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, Constants.kTimeoutMs);
        bRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, Constants.kTimeoutMs);
        
        fRight.setSelectedSensorPosition(0);
        bRight.setSelectedSensorPosition(0);
        fLeft.setSelectedSensorPosition(0);
        bLeft.setSelectedSensorPosition(0);

        fLeft.setSensorPhase(false);
        fRight.setSensorPhase(false);
        bLeft.setSensorPhase(false);
        bRight.setSensorPhase(false);

        fLeft.setInverted(false);
        fRight.setInverted(false);
        bLeft.setInverted(false);
        bRight.setInverted(false);

        fLeft.config_kP(kSlotIdx, Constants.drive_p, Constants.kTimeoutMs);
        fRight.config_kP(kSlotIdx, Constants.drive_p, Constants.kTimeoutMs);
        bLeft.config_kP(kSlotIdx, Constants.drive_p, Constants.kTimeoutMs);
        bRight.config_kP(kSlotIdx, Constants.drive_p, Constants.kTimeoutMs);

        fLeft.config_kI(kSlotIdx, Constants.drive_i, Constants.kTimeoutMs);
        fRight.config_kI(kSlotIdx, Constants.drive_i, Constants.kTimeoutMs);
        bLeft.config_kI(kSlotIdx, Constants.drive_i, Constants.kTimeoutMs);
        bRight.config_kI(kSlotIdx, Constants.drive_i, Constants.kTimeoutMs);

        fLeft.config_kD(kSlotIdx, Constants.drive_d, Constants.kTimeoutMs);
        fRight.config_kD(kSlotIdx, Constants.drive_d, Constants.kTimeoutMs);
        bLeft.config_kD(kSlotIdx, Constants.drive_d, Constants.kTimeoutMs);
        bRight.config_kD(kSlotIdx, Constants.drive_d, Constants.kTimeoutMs);

        fLeft.setNeutralMode((NeutralMode.Coast));
        fRight.setNeutralMode((NeutralMode.Coast));
        bLeft.setNeutralMode((NeutralMode.Coast));
        bRight.setNeutralMode((NeutralMode.Coast));
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void executeTank(double x, double y) {

    System.out.println(x);
    System.out.println(y);
    

    if (Math.abs(y) > 0.1)
        throttle = y;
    else
        throttle = 0.0;

    if (Math.abs(x) > 0.2)
        turn = x;
    else
        turn = 0.0;

        

    t_left = throttle + turn;
    t_right = throttle - turn;

    speedL = t_left + skim(t_right);
    speedR = t_right + skim(t_left);

    tank.tankDrive(speedL * .75, speedR * .75);

  }
  /**
   *  <p>Turns the robot at a given speed.</p>
   *  @param v Percent output 0-1 <em>(Positive values to turn right, negative values to turn left.)</em>
  */
  public void turn(double v) {
      // System.out.println("Workinggggggg TURN RIGHT");
      t_left = v;
      t_right = v;
      speedL = t_left + skim(t_right);
      speedR = t_right + skim(t_left);
      tank.tankDrive(speedL, speedR);
  }
  /**
   *  <p>Moves the robot at a given speed.</p>
   *  @param v Percent output 0-1 <em>(Positive values to move forward, negative values to move backward.)</em>
  */
  public void moveSpeed(double v) {
      t_left = -v;
      t_right = v;
      speedL = t_left + skim(t_right);
      speedR = t_right + skim(t_left);

      tank.tankDrive(speedL, speedR);
  }

  /**
   *  <p>Slowly turns the robot right.</p>
  */
  public void turnSlightlyRight() {
      t_left = 0.3;
      t_right = 0.3;
      speedL = t_left + skim(t_right);
      speedR = t_right + skim(t_left);
      tank.tankDrive(speedL, speedR);
  }

  /**
   *  <p>Slowly turns the robot left.</p>
  */
  public void turnSlightlyLeft() {
      t_left = -0.3;
      t_right = -0.3;
      speedL = t_left + skim(t_right);
      speedR = t_right + skim(t_left);
      tank.tankDrive(speedL, speedR);
  }


  private double skim(double v) {
      if (v > 1.0)
          return -((v - 1.0) * Constants.gain_skim);        
      else if (v < -1.0)
          return -((v + 1.0) * Constants.gain_skim);
      return 0;
  }

}
