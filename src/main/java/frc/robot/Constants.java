// 
// Constants used in the software build for FRC Team 6357's 2022 robot.
//

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    private Constants()
    {
        // Not called. Needed to prevent "Utility classes should not have a public or
        // default constructor" warning
    }

    /** Constants related to the ColorSensor */
    public static final class ColorSensor
    {
        public static final int MAXIMUM_PRESENCE_DISTANCE = 150;
        public static final int MINIMUM_PRSENCE_DISTANCE = 500;
    }

    /** Constants related to the Transfer Subsystem */
    public static final class TransferConstants
    {
        public static final int DISTANCE_THRESHOLD = 70;
        public static final double INTAKE_MOTOR_SPEED = 0.2;
        // Ejects the ball out of the robot
        public static final double EXIT_MOTOR_SPEED = 0.5;
        // Speed of the vertical section of the ball
        public static final double VERTICAL_MOTOR_SPEED = 1.0;

        public static final double BALL_EJECTION_SPEED = 0.5;
        // Uses the exit motor to move ball into the vertical portion
        public static final double LOAD_BALL_VERTICAL_SPEED = -0.1;

        public static final int TRANSFER_TO_VERTICAL_SHAFT_DURATION = 100;
        public static final int EJECT_DURATION = 100;

        public static final boolean EXIT_SENSOR_POLARITY = false;
        public static final boolean VERTICAL_SENSOR_POLARITY = false;
    }

    /** Constants related to Driving the Robot. */
    public static final class DriveConstants
    {
        public static final boolean LEFT_ENCODER_REVERSED  = false;
        public static final boolean RIGHT_ENCODER_REVERSED = true;

        public static final double                      TRACKWIDTH       = 0.635;   // Meters
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
                new DifferentialDriveKinematics(TRACKWIDTH);

        public static final double GEAR_REDUCTION             = 12.27;
        public static final int    ENCODER_CPR                = 2048;
        public static final double WHEEL_DIAMETER             = 0.1524;    // 6" in Meters
        public static final double ENCODER_DISTANCE_PER_PULSE =
                // Assumes the encoders are directly mounted on the wheel shafts
                (WHEEL_DIAMETER * Math.PI) / ((double) ENCODER_CPR * GEAR_REDUCTION);

        // Data taken from characterization analysis on 03/02/2021
        public static final double KS                = 0.651; // Volts
        public static final double KV                = 2.3;   // Volt Seconds Per Meter
        public static final double KA                = 0.217; // Volt Seconds Squared Per Meter
        public static final double KP_DRIVE_VELOCITY = 2.26;

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

        public static final PIDController          PID_CONTROLLER_LEFT           =
                new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0);
        public static final PIDController          PID_CONTROLLER_RIGHT           =
                new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0);
    }

    /** Constants related to the Launcher for the Robot. */
    public static final class LauncherConstants
    {
        public static final int BALL_LAUNCHER1 = 23;
        public static final int BALL_LAUNCHER2 = 24;

        public static final int CONTROLLER = 0;

        public static final int LOW_SPEED_PRESET   = 3000;
        public static final int MID_SPEED_PRESET   = 3400;
        public static final int HIGHT_SPEED_PRESET = 3600;
        public static final int MAX_SPEED_PRESET   = 4200;

        public static final double LAUNCHER_KP = 0.0005;
        public static final double LAUNCHER_KI = 0.0004;
        public static final double LAUNCHER_KD = 0.0;

        public static final int    LAUNCHER_ENCODER_CPR    = 2048;
        public static final double LAUNCH_GEAR_RATIO       = 24.0 / 16.0;
        public static final double LAUNCHER_WHEEL_DIAMETER = 0.18375;

        public static final boolean LEFT_ENCODER_REVERSED = false;

        public static final double ENCODER_DISTANCE_PER_PULSE =
                // Assumes the encoders are directly mounted on the wheel shafts
                (LAUNCHER_WHEEL_DIAMETER * Math.PI)
                    / ((double) LAUNCHER_ENCODER_CPR * LAUNCH_GEAR_RATIO);

        public static final double LAUNCHER_TRANSFER_SPEED = 1;
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
        public static final double TEST_DRIVE_STRAIGHT_1 = 100;    //Amount in centimeters
        public static final double TEST_DRIVE_STRAIGHT_2 = 200;    //Amount in centimeters
        public static final double TEST_DRIVE_STRAIGHT_3 = 500;    //Amount in centimeters

        public static final double TEST_TURN_1 = 10;     //Amount of degrees
        public static final double TEST_TURN_2 = 30;     //Amount of degrees
        public static final double TEST_TURN_3 = 50;     //Amount of degrees
        public static final double TEST_TURN_4 = 90;     //Amount of degrees
        public static final double TEST_TURN_5 = 180;    //Amount of degrees
    }

    /** Constants related to Climb */
    public static final class ClimbConstants
    {
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;

        public static final double ARM_MOTOR_RPM = 5000; //Guess???

        

        //Pivot
        public static final int PIVOT_MOTOR_TEETH_PER_REVOLUTION = 20; //Per De E
        public static final int PIVOT_MOTOR_GEAR_RATIO           = 25;  // 2x 5:1 Gearbox
        public static final int PIVOT_ARC_TEETH_PER_REVOLUTION   = 303; //59 teeth in a 70 degreee arc
        public static final double PIVOT_ARM_CONTROLLER_KP       = 0.005; // TODO: NEEDS TUNNING
        public static final double PIVOT_ARM_CONTROLLER_KI       = 0.0; // TODO: NEEDS TUNNING
        public static final double PIVOT_ARM_CONTROLLER_KD       = 0.0; // TODO: NEEDS TUNNING

        //Converts motor revs to degrees based on gearing
        public static final double PIVOT_MOTOR_ROTATIONS_TO_DEGREE_CONVERTER  =
                                            ((360.0 * PIVOT_MOTOR_TEETH_PER_REVOLUTION)
                                            / (PIVOT_MOTOR_GEAR_RATIO * PIVOT_ARC_TEETH_PER_REVOLUTION));
        

        //Lift
        public static final int LIFT_MOTOR_TEETH_PER_REVOLUTION = 20; // Check this!
        public static final int LIFT_RACK_TEETH_PER_INCH        = 20; // Check this!
        public static final int LIFT_MOTOR_GEAR_RATIO           = 9; // 9:1 gear ratio
        public static final double LIFT_ARM_CONTROLLER_KP       = 0.005; // TODO: NEEDS TUNNING
        public static final double LIFT_ARM_CONTROLLER_KI       = 0.0; // TODO: NEEDS TUNNING
        public static final double LIFT_ARM_CONTROLLER_KD       = 0.0; // TODO: NEEDS TUNNING

        //Converts motor revs to degrees based on gearing
        public static final double LIFT_MOTOR_ROTATIONS_TO_DEGREE_CONVERTER  =
                                            ((360.0 * PIVOT_MOTOR_TEETH_PER_REVOLUTION)
                                            / (PIVOT_MOTOR_GEAR_RATIO * PIVOT_ARC_TEETH_PER_REVOLUTION));

        public static final double COMPLEX_FULL_TILT            = 30;  //TODO: Needs tuning
        public static final double COMPLEX_FULL_STRAIGHTEN      = 0;
        public static final double COMPLEX_FULL_EXTEND          = 20;  //TODO: Needs Tuning
        public static final double COMPLEX_FULL_RETRACT         = 0;
        public static final double COMPLEX_PARTIAL_STRAIGHTEN   = 25;  //TODO: Needs Tuning
        public static final double COMPLEX_PARTIAL_EXTEND       = 5;   //TODO: Needs Tuning
        public static final long   STEP6_DELAY_MILLIS           = 1000; //TODO: Needs Tuning
        public static final long   STEP7_DELAY_MILLIS           = 1000; //TODO: Needs Tuning
        public static final long   STEP8_DELAY_MILLIS           = 1000; //TODO: Needs Tuning
        public static final long   STEP9_DELAY_MILLIS           = 1000; //TODO: Needs Tuning
        public static final long   STEP10_DELAY_MILLIS          = 1000; //TODO: Needs Tuning
        public static final long   STEP4_DELAY_MILLIS           = 1000; //TODO: Needs Tuning

        public static final double CURRENT_THRESHOLD            = 1000.0; //TODO: Needs Tuning
    }

    /**Constants related to Vision subsystem */
    public static final class VisionConstants
    {
        /** Represents the port on the Odroid-XU4 that packets will be sent from */
        public static final int ODROID_PORT         = 5005;

        /** Represents the port on the RoboRio that packets will be read from */
        public static final int ROBORIO_PORT        = 5800;

        /** This is the length of the packet received that is defined in the Datagram class
         *  within the SK22Vision subsystem file. This length must correspond to the same length
         *  defined on Odroid. */
        public static final int UDP_PACKET_LENGTH   = 29;

        /**
         * The tolerance for the angle in degrees for the robot to be considered to have
         * acquired the hub.
         */
        public static final double TARGET_ACQUIRED_TOLERANCE = 7.5;
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
