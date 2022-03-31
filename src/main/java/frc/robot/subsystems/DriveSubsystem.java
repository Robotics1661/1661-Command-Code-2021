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
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    AHRS m_gyro = new AHRS();

    private final DifferentialDriveOdometry m_odometry;

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
        
        // fRight.setSelectedSensorPosition(0);
        // bRight.setSelectedSensorPosition(0);
        // fLeft.setSelectedSensorPosition(0);
        // bLeft.setSelectedSensorPosition(0);

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

        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

        // fRight.configPulseWidthPeriod_EdgesPerRot(2048, 0);
        // fLeft.configPulseWidthPeriod_EdgesPerRot(2048, 0);

        resetEncoders();

        SmartDashboard.putNumber("gyro", 0.00);
        SmartDashboard.putNumber("velocity", 0.00);
        SmartDashboard.putNumber("acceleration", 0.00);
        SmartDashboard.putNumber("rEnc", 0.00);
        SmartDashboard.putNumber("lEnc", 0.00);
        SmartDashboard.putBoolean("gyroReset", false);
        SmartDashboard.putBoolean("rEncReset", false);
        SmartDashboard.putBoolean("lEncReset", false);
        SmartDashboard.putNumber("drivedraw", 0);

    
  }

  @Override
  public void periodic() {

    System.out.println("Right dist: " + (fRight.getSelectedSensorPosition() / 2048) * 2 * Math.PI * Constants.wheel_radius);
    System.out.println("Left dist: " + (-fLeft.getSelectedSensorPosition() / 2048) * 2 * Math.PI * Constants.wheel_radius);
    // This method will be called once per scheduler run
    m_odometry.update(
        m_gyro.getRotation2d(), (fRight.getSelectedSensorPosition() / 2048) * 2 * Math.PI * Constants.wheel_radius, (-fLeft.getSelectedSensorPosition() / 2048) * 2 * Math.PI * Constants.wheel_radius);

    /**
		 * In this section, the variables that were initialized in robotInit are continuously updated and read from.
		 * This if statement checks if the gyroReset button was clicked on the dashboard, and reset the gyro if so.
		 */
		if (SmartDashboard.getBoolean("gyroReset", false)) {
			m_gyro.reset();
			SmartDashboard.putBoolean("gyroReset", false);
		}
        if (SmartDashboard.getBoolean("rEncReset", false)) {
			fRight.setSelectedSensorPosition(0);
			SmartDashboard.putBoolean("rEncReset", false);
		}
        if (SmartDashboard.getBoolean("lEncReset", false)) {
			fLeft.setSelectedSensorPosition(0);
			SmartDashboard.putBoolean("lEncReset", false);
		}

    double totalDriveCurrentDraw = Math.abs(fLeft.getSupplyCurrent()) + Math.abs(fRight.getSupplyCurrent()) + Math.abs(bLeft.getSupplyCurrent()) + Math.abs(bRight.getSupplyCurrent());

    SmartDashboard.putNumber("gyro", m_gyro.getAngle());
    SmartDashboard.putNumber("acceleration", m_gyro.getAccelFullScaleRangeG());
    SmartDashboard.putNumber("rEnc", fRight.getSelectedSensorPosition());
    SmartDashboard.putNumber("lEnc", fLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("driveDraw", totalDriveCurrentDraw);
    double avgVel = (fRight.getSelectedSensorVelocity() + -fLeft.getSelectedSensorVelocity()) / 2;
    SmartDashboard.putNumber("velocity", avgVel);
    // System.out.println("Right: " + fRight.getSelectedSensorVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(fRight.getSelectedSensorVelocity(), -fLeft.getSelectedSensorVelocity());
  }

    /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

    /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
    tank.feed();
  }

  public void stop() {
    fRight.set(0);
    bRight.set(0);
    fLeft.set(0);
    bLeft.set(0);
  }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        fRight.setSelectedSensorPosition(0);
        bRight.setSelectedSensorPosition(0);
        fLeft.setSelectedSensorPosition(0);
        bLeft.setSelectedSensorPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
    return (fRight.getSelectedSensorPosition() + fLeft.getSelectedSensorPosition()) / 2.0;
    }

    /**
 * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
 *
 * @param maxOutput the maximum output to which the drive will be constrained
 */
    public void setMaxOutput(double maxOutput) {
        tank.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }

  public void executeTank(double x, double y) {

    // System.out.println(x);
    // System.out.println(y);
    

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
      t_left = 0.4;
      t_right = 0.4;
      speedL = t_left + skim(t_right);
      speedR = t_right + skim(t_left);
      tank.tankDrive(speedL, speedR);
  }

  /**
   *  <p>Slowly turns the robot left.</p>
  */
  public void turnSlightlyLeft() {
      t_left = -0.4;
      t_right = -0.4;
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
