// 
// Definitions of addresses and hardware port numbers used in
// the FRC Team 6357 2022 robot.
//
package frc.robot;

/**
 * Definitions of all hardware connections and hardware-related IDs on the robot. This
 * class should be included in any other class which needs to interact with the robot
 * hardware. The values here form part of the robot's control system configuration
 * specification.
 */
public class Ports
{
    //////////////////////////////
    // Drivetrain Addresses
    //////////////////////////////
    public static final int FRONT_LEFT_DRIVE_MOTOR      = 10; // CAN ID
    public static final int FRONT_RIGHT_DRIVE_MOTOR     = 12; // CAN ID
    public static final int BACK_LEFT_DRIVE_MOTOR       = 11; // CAN ID
    public static final int BACK_RIGHT_DRIVE_MOTOR      = 13; // CAN ID

    public static final int FRONT_LEFT_ANGLE_MOTOR      = 20; // CAN ID
    public static final int FRONT_RIGHT_ANGLE_MOTOR     = 22; // CAN ID
    public static final int BACK_LEFT_ANGLE_MOTOR       = 21; // CAN ID
    public static final int BACK_RIGHT_ANGLE_MOTOR      = 23; // CAN ID

    ///////////////////////////////
    // Swerve direction encoders
    ///////////////////////////////
    public static final int FRONT_LEFT_ANGLE_ENCODER    = 30; // CAN ID
    public static final int FRONT_RIGHT_ANGLE_ENCODER   = 32; // CAN ID
    public static final int BACK_LEFT_ANGLE_ENCODER     = 31; // CAN ID
    public static final int BACK_RIGHT_ANGLE_ENCODER    = 33; // CAN ID
    
    ///////////////////////////////
    // Operator Interface
    ///////////////////////////////

    // Joystick Ports
    public static final int OI_DRIVE_CONTROLLER         = 0;
    public static final int OI_ANGLE_CONTROLLER         = 1;

    // This example uses a thumbstick axis to control each individual motor.
    // The drive controller operates the 4 swerve module drive motors and the
    // turn controller operates the turn direction motors.

    // Drive Controls
    public static final int OI_DRIVE_FRONT_LEFT         = 0;  // Left joystick X
    public static final int OI_DRIVE_FRONT_RIGHT        = 4;  // Right joystick X
    public static final int OI_DRIVE_BACK_LEFT          = 1;  // Left joystick Y
    public static final int OI_DRIVE_BACK_RIGHT         = 5;  // Right joystick Y

    // The Y axes report inverted values - pushing the stick forward reduces
    // the output and pulling it back increases it. This feels backwards so we
    // define the following to allow the code to determine whether it needs to
    // negate each joystick channel after reading it.
    public static final Boolean NEGATE_FRONT_LEFT_DRIVE_AXIS     = false;
    public static final Boolean NEGATE_FRONT_RIGHT_DRIVE_AXIS    = false;
    public static final Boolean NEGATE_BACK_LEFT_DRIVE_AXIS      = true;
    public static final Boolean NEGATE_BACK_RIGHT_DRIVE_AXIS     = true;

    // Turn Controls
    public static final int OI_ANGLE_FRONT_LEFT         = 0;  // Left joystick X
    public static final int OI_ANGLE_FRONT_RIGHT        = 4;  // Right joystick X
    public static final int OI_ANGLE_BACK_LEFT          = 1;  // Left joystick Y
    public static final int OI_ANGLE_BACK_RIGHT         = 5;  // Right joystick Y

    // Determine which (if any) of the joystick axes should be inverted.
    public static final Boolean NEGATE_FRONT_LEFT_ANGLE_AXIS     = false;
    public static final Boolean NEGATE_FRONT_RIGHT_ANGLE_AXIS    = false;
    public static final Boolean NEGATE_BACK_LEFT_ANGLE_AXIS      = false;
    public static final Boolean NEGATE_BACK_RIGHT_ANGLE_AXIS     = false;   
}
