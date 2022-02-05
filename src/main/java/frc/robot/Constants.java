// 
// Constants used in the software build for FRC Team 6357's 2022 robot.
//

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place to hold robot-wide numerical
 * or boolean constants.
 */
public final class Constants
{

    /** Constants related to the Transfer Subsystem */
    public static final class TransferConstants{
        public static final int DISTANCE_THRESHOLD = 70;
        public static final double POSITION_ONE_MOTOR_SPEED = 0.2;
        public static final double POSITION_TWO_MOTOR_SPEED = 0.2;
        public static final double POSITION_THREE_MOTOR_SPEED = 0.2;
        public static final double STOP_SPEED = 0.0;
        public static final double BALL_EJECTION_SPEED = 0.5;
        public static final double BALL_VERTICAL_LOAD_SPEED = 0.5;
    }
    /** Constants related to Driving the Robot. */
    public static final class DriveConstants
    {
        public static final boolean LEFT_ENCODER_REVERSED   = false;
        public static final boolean RIGHT_ENCODER_REVERSED  = true;

        public static final double TRACKWIDTH           = 0.69;   // Meters
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
            new DifferentialDriveKinematics(TRACKWIDTH);

        public static final double GEAR_REDUCTION       = 12.412;
        public static final int    ENCODER_CPR          = 2048;
        public static final double WHEEL_DIAMETER       = 0.18375;    // Meters
        public static final double ENCODER_DISTANCE_PER_PULSE =
            // Assumes the encoders are directly mounted on the wheel shafts
            (WHEEL_DIAMETER * Math.PI) / ((double) ENCODER_CPR * GEAR_REDUCTION);

        // Data taken from characterization analysis on 03/02/2021
        public static final double KS                   = 0.651; // Volts
        public static final double KV                   = 2.3;   // Volt Seconds Per Meter
        public static final double KA                   = 0.217; // Volt Seconds Squared Per Meter
        public static final double KP_DRIVE_VELOCITY    = 2.26;  // TODO: Check units for this value

        public static final double  DEADBAND_TURN       = 0.01;
        public static final double  SLEW_FILTER_RATE    = 2;     // per second

        public static final double TURN_TOLERANCE       = 5;    // Degrees
        public static final double TURN_RATE_TOLERANCE  = 5;    // Degrees per Second
    }

    /** Constants related to the Autonomous operation mode for the Robot. */
    public static final class AutoConstants
    {
        public static final double MAX_SPEED            = 2;     // Meters Per Second
        public static final double MAX_ACCELERATION     = 0.50;  // Meters Per Seconds Squared

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double RAMSETE_B            = 2;
        public static final double RAMSETE_ZETA         = 0.7;
    }

    /** Constants related to the Launcher for the Robot. */
    public static final class LauncherConstants {
        public static final int BALL_LAUNCHER1               = 23;
        public static final int BALL_LAUNCHER2               = 24;

        public static final int CONTROLLER                  = 0;

        public static final int LOW_SPEED_PRESET              = 3000;
        public static final int MID_SPEED_PRESET              = 3400;
        public static final int HIGHT_SPEED_PRESET             = 3600;
        public static final int MAX_SPEED_PRESET              = 4200;

        public static final double LAUNCHER_KP              = 0.0005;
        public static final double LAUNCHER_KI              = 0.0004;
        public static final double LAUNCHER_KD              = 0.0;

        public static final int LAUNCHER_ENCODER_CPR        = 2048;
        public static final double LAUNCH_GEAR_RATIO        = 24.0/16.0;
        public static final double LAUNCHER_WHEEL_DIAMETER  = 0.18375;

        public static final boolean LEFT_ENCODER_REVERSED   = false;

        public static final double ENCODER_DISTANCE_PER_PULSE =
            // Assumes the encoders are directly mounted on the wheel shafts
            (LAUNCHER_WHEEL_DIAMETER * Math.PI) / ((double) LAUNCHER_ENCODER_CPR * LAUNCH_GEAR_RATIO);
    }

    /** Constants related to Intake */
    public static final class IntakeConstants 
    {
        // TODO: Tune this value
        public static final double INTAKE_MOTOR_SPEED = 0.75;
    }

    /** Constants related to Test Mode */
    public static final class TestConstants
    {
        public static final double TEST_DRIVE_STRAIGHT_1        = 100;    //Amount in centimeters
        public static final double TEST_DRIVE_STRAIGHT_2        = 200;    //Amount in centimeters
        public static final double TEST_DRIVE_STRAIGHT_3        = 500;    //Amount in centimeters

        public static final double TEST_TURN_1                  = 10;     //Amount of degrees
        public static final double TEST_TURN_2                  = 30;     //Amount of degrees
        public static final double TEST_TURN_3                  = 50;     //Amount of degrees
        public static final double TEST_TURN_4                  = 90;     //Amount of degrees
        public static final double TEST_TURN_5                  = 180;    //Amount of degrees
    }

    /** Constants related to Climb */
    public static final class ClimbConstants
    {
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        
        public static final double ARM_MOTOR_RPM = 5000; //Guess???

        //Pivot
        public static final int PIVOT_MOTOR_TEETH_PER_REVOLUTION = 20; //Per De E
        public static final int PIVOT_MOTOR_GEAR_RATIO = 25;  // 2x 5:1 Gearbox
        public static final int PIVOT_ARC_TEETH_PER_REVOLUTION = 120; //Per De E

        //Lift
        public static final int LIFT_MOTOR_TEETH_PER_REVOLUTION = 20; // Check this!
        public static final int LIFT_RACK_TEETH_PER_INCH = 20; // Check this!
        public static final int LIFT_MOTOR_GEAR_RATIO = 9; // 9:1 gear ratio
    }

    public static final double INCH_PER_MILLIMETER = 0.0393701;
    public static final int SIM_TICS_PER_SECOND = 50;
    public static final int SECONDS_PER_MINUTE = 60;
    public static final int DEGREES_PER_REVOLUTION = 360;
  
    public static final double TRIGGER_THRESHOLD = 0.75;

    public static final String SPLINE_DIRECTORY = "paths/output";
    public static final String AUTOS_FOLDER_DIRECTORY = "paths/Autos";
    public static final String SUBSYSTEM = "Subsystems.json";

}
