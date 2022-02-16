// 
// Constants used in the software build for FRC Team 6357's 2022 robot.
//

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place to hold robot-wide numerical or boolean
 * constants.
 */
public final class Constants
{
    /** Constants related to Driving the Robot. */
    public static final class DriveConstants
    {
        public static final boolean LEFT_ENCODER_REVERSED  = false;
        public static final boolean RIGHT_ENCODER_REVERSED = true;

        public static final double                      TRACKWIDTH       = 0.69;   // Meters
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
                new DifferentialDriveKinematics(TRACKWIDTH);

        public static final double GEAR_REDUCTION             = 12.412;
        public static final int    ENCODER_CPR                = 2048;
        public static final double WHEEL_DIAMETER             = 0.18375;    // Meters
        public static final double ENCODER_DISTANCE_PER_PULSE =
                // Assumes the encoders are directly mounted on the wheel shafts
                (WHEEL_DIAMETER * Math.PI) / ((double) ENCODER_CPR * GEAR_REDUCTION);

        // Data taken from characterization analysis on 03/02/2021
        public static final double KS                = 0.651; // Volts
        public static final double KV                = 2.3;   // Volt Seconds Per Meter
        public static final double KA                = 0.217; // Volt Seconds Squared Per Meter
        public static final double KP_DRIVE_VELOCITY = 2.26;  // TODO: Check units for this value

        public static final double DEADBAND_TURN    = 0.01;
        public static final double SLEW_FILTER_RATE = 2;     // per second

        public static final double TURN_TOLERANCE      = 5;    // Degrees
        public static final double TURN_RATE_TOLERANCE = 5;    // Degrees per Second
    }

    /** Constants related to the Autonomous operation mode for the Robot. */
    public static final class AutoConstants
    {
        public static final double MAX_SPEED        = 2;     // Meters Per Second
        public static final double MAX_ACCELERATION = 0.50;  // Meters Per Seconds Squared

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double RAMSETE_B    = 2;
        public static final double RAMSETE_ZETA = 0.7;

        public static final RamseteController      RAMSETE_CONTROLLER       =
                new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA);
        public static final SimpleMotorFeedforward SIMPLE_MOTOR_FEEDFORWARD =
                new SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV, DriveConstants.KA);
        public static final PIDController          PID_CONTROLLER           =
                new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0);
    }

    public static final double INCH_PER_MILLIMETER    = 0.0393701;
    public static final int    SIM_TICS_PER_SECOND    = 50;
    public static final int    SECONDS_PER_MINUTE     = 60;
    public static final int    DEGREES_PER_REVOLUTION = 360;

    public static final double TRIGGER_THRESHOLD = 0.75;

    public static final String SPLINE_DIRECTORY       = "paths/output";
    public static final String AUTOS_FOLDER_DIRECTORY = "paths/Autos";
    public static final String SUBSYSTEM              = "Subsystems.json";

}
