package frc.robot;

/**
 * TuningParams represents Tuning Parameters for the Robot.
 * 
 * These are generally items that are dependent upon a specific robot design, not
 * generally usable for another robot.
 */
public final class TuningParams 
{
    //////////////////////////////
    // Drive Params
    //////////////////////////////
    public static final double  ACCEL_MAX_TOWARDS_FORWARD       =  0.125;
    public static final double  ACCEL_MAX_TOWARDS_BACKWARD      = -0.125;
    public static final double  DECEL_MAX_TOWARDS_FORWARD       =  0.125;
    public static final double  DECEL_MAX_TOWARDS_BACKWARD      = -0.125;

    public static final double  CONTROLLED_TURN_SPEED           = 0.5;
    public static final double  SLOW_CONTROLLED_TURN_SPEED      = 0.2;
    public static final double  ANGLE_TURN_TOLERANCE            = 1;
    public static final double  AUTONOMOUS_SLOW_START_ANGLE     = 30;       // In Degrees

    public static final boolean ENCODER_LEFT_REVERSED           = false;
    public static final boolean ENCODER_RIGHT_REVERSED          = true;
    public static final int     ENCODER_PULSES                  = 480;
    public static final double  DRIVETRAIN_GEAR_REDUCTION       = 12.412;
    public static final double  WHEEL_DIAMETER                  = 18.375;   // 7.5" In Centimeters
    public static final double  WHEEL_CIRCUMFERENCE             = 57.72;

    public static final double  STRAIGHT_DRIVE_OFFSET_TOLERANCE = 1.0;      // In Centimeters
    public static final double  OFFSET_SPEED_INCREMENT          = 0.01;
    public static final double  AUTONOMOUS_DRIVE_SPEED          = 0.5;
    public static final double  AUTONOMOUS_LOW_SPEED_LEVEL      = 0.2;
    public static final double  AUTONOMOUS_SLOW_DISTANCE_AREA   = 40;       // In Centimeters

    public static final double  DRIVETRAIN_KP                   = 0.00000485;
    public static final double  DRIVETRAIN_KD                   = 0;

    public static final double  SLOWMODE_MULTIPLIER             = 0.25;
    public static final double  SLOWMODE_TRIGGER_THRESHOLD      = 0.75;     // Joystick Trigger Required Value

    public static final double  DEADBAND_TURN                   = 0.01;

    ///////////////////////////////
    // Auto Params
    ///////////////////////////////
    public static final double AUTO_STRAIGHTSTMOVE_DRIVE_DISTANCE       = 50.0; //cm
    public static final double AUTO_OFFSETSTMOVE_DRIVE_DISTANCE         = -1000.0;
    public static final double LAUNCHER_START_UP_TIME                   = 3.0;

    public static final int AUTO_STRAIGHTSTTRENCH_TURN_ANGLE            = 90;
    public static final double AUTO_STRAIGHTSTTRENCH_DRIVE_DISTANCE_1   = 192.024;

    public static final double AUTO_TRENCH_DRIVE_DISTANCE               = 670.56;

    public static final int AUTO_OFFSETSTTRENCH_TURN_ANGLE_1            = -148;

    // Distance to drive forward or backward to get off the line in "Drive forwards" and "Drive backwards"
    public static final double AUTO_DRIVE_DISTANCE                      = 100.0;
    public static final double AUTO_DRIVE_DISTANCE_TO_WALL              = 294.0;   //10ft start line to wall
    public static final double AUTO_FIRE_ALL_BALLS_TIME                 = 4000.0;  // Milliseconds

    ///////////////////////////////
    // Test Params
    ///////////////////////////////
    public static final double TEST_DRIVE_STRAIGHT_1        = 100;    //Amount in centimeters
    public static final double TEST_DRIVE_STRAIGHT_2        = 200;    //Amount in centimeters
    public static final double TEST_DRIVE_STRAIGHT_3        = 500;    //Amount in centimeters

    public static final double TEST_TURN_1                  = 10;    //Amount of degrees
    public static final double TEST_TURN_2                  = 30;    //Amount of degrees
    public static final double TEST_TURN_3                  = 50;    //Amount of degrees
    public static final double TEST_TURN_4                  = 90;    //Amount of degrees
    public static final double TEST_TURN_5                  = 180;    //Amount of degrees

    public static final String SPLINE_DIRECTORY = "C:/Users/Owner/Documents/WeaverOutput/output";
}
