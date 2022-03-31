// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class UltrasonicSubsystem extends SubsystemBase {

    private final AnalogInput ultrasonicSensor = new AnalogInput(0);

  public UltrasonicSubsystem() {

    SmartDashboard.putNumber("ultrasonic", 0.00);
   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentDistanceInches = getDistIn();
    SmartDashboard.putNumber("Current Distance (inches)", currentDistanceInches);
    SmartDashboard.putNumber("ultrasonic", currentDistanceInches);


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

// GET DISTANCES FROM ULTRASONIC SENSOR
    /**
     *  <p>Gets the distance from the ultrasonic sensor in centimeters.</p>
    */
    
    public double getDistCm() {
        double raw_value = ultrasonicSensor.getValue();
        
        //voltage_scale_factor allows us to compensate for differences in supply voltage.
        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        double currentDistanceCentimeters = raw_value * voltage_scale_factor * 0.125;
        return currentDistanceCentimeters;
    }
    // GET DISTANCES FROM ULTRASONIC SENSOR
    /**
     *  <p>Gets the distance from the ultrasonic sensor in inches.</p>
    */
    public double getDistIn() {
        double raw_value = ultrasonicSensor.getValue();

        //voltage_scale_factor allows us to compensate for differences in supply voltage.
        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        double currentDistanceInches = raw_value * voltage_scale_factor * 0.0492;
        return currentDistanceInches;
    }

}
