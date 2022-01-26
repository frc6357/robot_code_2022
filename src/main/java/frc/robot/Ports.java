package frc.robot;

// Resource Allocations:
//
// PCM 0 - colorSpinnerExtend
//     1 - launcherHoodExtend
//     2 - intakeMoverRaise  
//     3 - launcherFeederRetract
//     4 - launcherFeederExtend 
//     5 - intakeMoverDrop 
//     6 - launcherHoodRetract      
//     7 - colorSpinnerRetract

/** 
 * Definitions of all hardware connections and hardware-related
 * IDs on the robot. This class should be included in any other
 * class which needs to interact with the robot hardware. The
 * values here form part of the robot's control system configuration
 * specification.
 */
public class Ports 
{

    //////////////////////////////
    // Robot Infrastructure
    //////////////////////////////
    public static final int pcm                         = 1;  // CAN ID

    //////////////////////////////
    // I2C device addresses
    //////////////////////////////
    public static final int i2cColorSensor              = 0x52;

    //////////////////////////////
    // Drivetrain Addresses
    //////////////////////////////
    public static final int frontLeftDrive              = 10; // CAN ID
    public static final int frontRightDrive             = 11; // CAN ID
    public static final int backLeftDrive               = 12; // CAN ID
    public static final int backRightDrive              = 13; // CAN ID

    //////////////////////////////
    // Intake Addresses
    //////////////////////////////
    

    ///////////////////////////////
    // Ball Indexer Addresses
    ///////////////////////////////
    

    ///////////////////////////////
    // Ball Launcher Addresses
    ///////////////////////////////
    public static final int ballLauncher1           = 23; // CAN ID
    public static final int ballLauncher2            = 24; // CAN ID

    ///////////////////////////////
    // Climb Addresses
    ///////////////////////////////


    ///////////////////////////////
    // Additional hardware
    ///////////////////////////////
    public static final int MindSensorsCANLight         = 3;  // CAN ID of LED strip controller

    ///////////////////////////////
    // Operator Interface
    ///////////////////////////////
    
    // Joystick Ports
    public static final int OIDriverLeftJoystick        = 0;
    public static final int OIDriverRightJoystick       = 1;

    // Controls set for Arcade Drive - left stick turn, right stick throttle.
    public static final int OIDriverTurn                = 2;  // Z Axis for Extreme 3D Pro
    public static final int OIDriverMove                = 1;  // Y Axis for Extreme 3D Pro

    // Controls set for Tank Drive
    public static final int OIDriverSpeedAxis           = 1;           
    
    public static final int OIDriverSlowmode            = 1;  // Right Trigger Axis    
}
