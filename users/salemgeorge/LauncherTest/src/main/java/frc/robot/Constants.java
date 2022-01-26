// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

// TODO: The values in this file represent parameters for Banshee 2.0. These will need
// to be updated for the 2022 robot.

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical
 * or boolean constants. This class should not be used for any other purpose. All
 * constants should be declared globally (i.e. public static). Do not put anything
 * functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever
 * the constants are needed, to reduce verbosity.
 */
public final class Constants
{
    /**
     * Constants related to Driving the Robot.
     */
    public static final class DriveConstants
    {
        public static final boolean LEFT_ENCODER_REVERSED   = false;
        public static final boolean RIGHT_ENCODER_REVERSED  = true;

        // TODO: The value here is what we measured from the robot. Check to make sure this works
        // correctly because the drive characterization tool determines the track width automatically
        // and comes up with a rather different value closer to 1.0m.
        public static final double TRACKWIDTH = 0.69;   // Meters
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
            new DifferentialDriveKinematics(TRACKWIDTH);

        public static final double GEAR_REDUCTION = 12.412;
        public static final int    ENCODER_CPR = 2048;
        public static final double WHEEL_DIAMETER = 0.18375;    // Meters
        public static final double ENCODER_DISTANCE_PER_PULSE =
            // Assumes the encoders are directly mounted on the wheel shafts
            (WHEEL_DIAMETER * Math.PI) / ((double) ENCODER_CPR * GEAR_REDUCTION);

        // Data taken from characterization analysis on 01/21/2022
        public static final double KS = 0.47667;        // Volts
        public static final double KV = 0.0037589;      // Volt Seconds Per Meter
        public static final double KA = 0.00043482;     // Volt Seconds Squared Per Meter
        public static final double KP_DRIVE_VELOCITY = 2.26; // TODO: Check units for this value

        public static final double  DEADBAND_TURN = 0.01;
    }

    /**
     * Constants related to the Autonomous operation mode for the Robot.
     */
    public static final class AutoConstants
    {
        public static final double MAX_SPEED = 2;   // Meters Per Second
        public static final double MAX_ACCELERATION = .50;  // Meters Per Seconds Squared

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;
    }

    /**
     * Constants related to Test Mode
     */
    public static final class TestConstants
    {
        public static final double TEST_DRIVE_STRAIGHT_1        = 100;    //Amount in centimeters
        public static final double TEST_DRIVE_STRAIGHT_2        = 200;    //Amount in centimeters
        public static final double TEST_DRIVE_STRAIGHT_3        = 500;    //Amount in centimeters

        public static final double TEST_TURN_1                  = 10;    //Amount of degrees
        public static final double TEST_TURN_2                  = 30;    //Amount of degrees
        public static final double TEST_TURN_3                  = 50;    //Amount of degrees
        public static final double TEST_TURN_4                  = 90;    //Amount of degrees
        public static final double TEST_TURN_5                  = 180;    //Amount of degrees
    }

    public static final class LauncherConstants {
        public static final int ballLauncher1 = 4;
        public static final int ballLauncher2 = 5;

        public static final int controller = 0;
        
        public static final int lowSpeedPresetButton = 1;
        public static final int maxSpeedPresetButton = 2;
        public static final int midSpeedPresetButton = 3;
        public static final int highSpeedPresetButton = 4;
        public static final int enableLauncherButton = 5;
        public static final int disableLauncherButton = 6;

        public static final int lowSpeedPreset = 2500;
        public static final int midSpeedPreset = 5000;
        public static final int highSpeedPreset = 7500;
        public static final int maxSpeedPreset = 10000;
    }

    public static final double INCH_PER_MILLIMETER = 0.0393701;
  
    public static final String SPLINE_DIRECTORY = "paths/output";
    public static final String SUBSYSTEM = "Subsystems.json";

}
