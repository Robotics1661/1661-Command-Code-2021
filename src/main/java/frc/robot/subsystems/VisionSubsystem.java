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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;

public class VisionSubsystem extends SubsystemBase {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    OI oi = new OI();

  public VisionSubsystem() {

    SmartDashboard.putNumber("LimelightY", 0.00);
    SmartDashboard.putBoolean("onTarget", false);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    // System.out.println(y);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    boolean onTarget = area != 0 && Math.abs(x) < 3.0;
    boolean inRange = y > 8.7 && y < 9.1;
    SmartDashboard.putBoolean("onTarget", onTarget);

    if (onTarget && inRange) {
      oi.m_mechController.setRumble(RumbleType.kRightRumble, .5);
      oi.m_mechController.setRumble(RumbleType.kLeftRumble, .5);
    } else {
      oi.m_mechController.setRumble(RumbleType.kRightRumble, 0);
      oi.m_mechController.setRumble(RumbleType.kLeftRumble, 0);
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

public double getLimelightX() {
    return tx.getDouble(0.0);
}

public double getLimelightValid() {
    return tv.getDouble(0.0);
}

}
