// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static int front_right_drive = 5;
    public final static int front_left_drive = 6;
    public final static int back_right_drive = 7; 
    public final static int back_left_drive = 8;

    public final static int shooter_motor = 19;

    public final static int intake_motor = 10; 

    public static final int horizontal_agitator_right = 16;
    public static final int horizontal_agitator_left = 17;

    public final static int vertical_agitator_right = 11;
    public final static int vertical_agitator_left = 12;
    public final static int vertical_agitator_front = 13;


    // Drive

    public final static double gain_skim = 0.9;
    public final static double gain_turn = 1.0;

    public final static double feet_to_meters = 0.3048;

    public final static double time_step = 0.05; // s

    public final static double max_vel = 1; // m/s

    public final static double torque_per_motor = 2.425; // N*m
    public final static int num_motors = 4;
    public final static double low_gear_ratio = 0.115;
    public final static double high_gear_ratio = 0.277;

    public final static double torque = torque_per_motor * num_motors * high_gear_ratio; // N*m

    public final static double wheel_radius = 0.0762; // m
    public final static double robot_weight = 45; // kg
    public final static double max_accel = torque / (wheel_radius * robot_weight); // m/s/s

    public final static double encoder_units_per_rotation = 2048 * 2; // might be 4096 - TEST & CHECK

    public final static double max_jerk = 60.0; // m/s/s/s

    public final static double wheelbase_width = 28.0 / 12 * feet_to_meters;

    public final static int continuous_current = 60;
    public final static int peak_current = 60;
    public final static double open_ramp = 0.2;

    public static final double ksVolts = 0.67962;
    public static final double kvVoltSecondsPerMeter = 0.10272;
    public static final double kaVoltSecondsSquaredPerMeter = 0.020621;

    public static final double kPDriveVel = 0.14956;
    public static final double kTrackwidthMeters = 0.635;

    public static final double kMaxSpeedMetersPerSecond = .2;
    public static final double kMaxAccelerationMetersPerSecondSquared = .2;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    
    // PID

    public static final double degreesToTicks = 4096./360;
    public static final int kTimeoutMs = 10; // Re-evaluate this
    public static final int ticks_per_revolution = 4096;

    
    // Drive PID

    public static final double drive_p = 0.3;
    public static final double drive_i = 0.0;
    public static final double drive_d = 0.0;
    public static final double drive_f = 0.0;

    
    // Control

    public final static int left_x_axis = 0;
    public final static int left_y_axis = 1;

    public final static int right_x_axis = 4;
    public final static int right_y_axis = 5;

    public final static int left_trigger = 2;
    public final static int right_trigger = 3;

    public final static int a_button = 1;
    public final static int b_button = 2;
    public final static int x_button = 3;
    public final static int y_button = 4;
    public final static int left_bumper = 5;
    public final static int right_bumper = 6;
    public final static int back_button = 7;
    public final static int start_button = 8;
    public final static int left_stick_button = 9;
    public final static int right_stick_button = 10;

}
