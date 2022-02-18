// 
// Definitions of addresses and hardware port numbers used in
// the FRC Team 6357 2022 robot.
//
package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

//
// Pneumatic Resource Allocations:
//
// PCM 0 port 0 - intake forward
//     0 port 1 - intake reverse
//     0 port 2 - gear shift high
//     0 port 3 - gear shift low
//     0 port 4 - climb hand forward
//     0 port 5 - climb hand reverse
//     0 port 6 - climb complexBrakePiston forward
//     0 port 7 - climb complexBrakePiston reverse
//     0 port 8 - climb complexRatchetPiston forward
//     0 port 9 - climb complexRatchetPiston reverse

/**
 * Definitions of all hardware connections and hardware-related IDs on the robot. This
 * class should be included in any other class which needs to interact with the robot
 * hardware. The values here form part of the robot's control system configuration
 * specification.
 */
public class Ports
{

    //////////////////////////////
    // Robot Infrastructure
    //////////////////////////////
    public static final int BASE_PCM               = 1;  // CAN ID
    public static final int CLIMB_PNEUMATIC_MODULE = 2;

    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;

    //////////////////////////////
    // I2C device addresses
    //////////////////////////////
    public static final Port I2C_COLOR_SENSOR = I2C.Port.kOnboard;

    //////////////////////////////
    // Drivetrain Addresses
    //////////////////////////////
    public static final int FRONT_LEFT_DRIVE  = 11; // CAN ID
    public static final int FRONT_RIGHT_DRIVE = 13; // CAN ID
    public static final int BACK_LEFT_DRIVE   = 10; // CAN ID
    public static final int BACK_RIGHT_DRIVE  = 12; // CAN ID

    public static final int GEAR_SHIFT_HIGH = 2;
    public static final int GEAR_SHIFT_LOW  = 3;

    //////////////////////////////
    // Transfer Addresses
    //////////////////////////////
    public static final int EXIT_TRANSFER_MOTOR         = 31; // Currently unknown Port
    public static final int INTAKE_TRANSFER_MOTOR       = 30; // Currently unknown Port
    public static final int VERTICAL_TRANSFER_MOTOR     = 12; // Currently unknown Port 
    public static final int LAUNCHER_TRANSFER_MOTOR     = 12; // Currently unknown Port
    // TODO: Find out what these ports are
    public static final int EXIT_SENSOR                 = 0;  // Currently unknown Port
    public static final int VERTICAL_SENSOR             = 1;  // Currently unknown Port

    //////////////////////////////
    // Intake Addresses
    //////////////////////////////
    public static final int INTAKE_MOTOR = 31; // CAN ID 

    public static final int INTAKE_MOVER_FORWARD  = 0;  // PCM Channel 
    public static final int INTAKE_MOVER_BACKWARD = 1;  // PCM Channel 

    public static final PneumaticsModuleType INTAKE_PCM = PneumaticsModuleType.CTREPCM;

    ///////////////////////////////
    // Ball Indexer Addresses
    ///////////////////////////////

    ///////////////////////////////
    // Ball Launcher Addresses
    ///////////////////////////////
    public static final int BALL_LAUNCHER_1 = 23; // CAN ID
    public static final int BALL_LAUNCHER_2 = 24; // CAN ID

    ///////////////////////////////
    // Climb Addresses
    ///////////////////////////////
    public static final int COMPLEX_BRAKE_PIVOT                          = 1; // Currently unknown Port
    public static final int COMPLEX_RATCHET_LIFT                         = 2; // Currently unknown Port
    public static final int COMPLEX_CLIMB_RATCHET_PISTON_FORWARD_CHANNEL = 8;
    public static final int COMPLEX_CLIMB_RATCHET_PISTON_REVERSE_CHANNEL = 9;
    public static final int SIMPLE_CLIMB_TILT_PISTON_FORWARD_CHANNEL     = 10;
    public static final int SIMPLE_CLIMB_TILT_PISTON_REVERSE_CHANNEL     = 11;
    public static final int SIMPLE_CLIMB_LIFT_PISTON_FORWARD_CHANNEL     = 12;
    public static final int SIMPLE_CLIMB_LIFT_PISTON_REVERSE_CHANNEL     = 13;

    ///////////////////////////////
    // Additional hardware
    ///////////////////////////////
    public static final int MindSensorsCANLight = 3;  // CAN ID of LED strip controller

    ///////////////////////////////
    // Operator Interface
    ///////////////////////////////

    // Verified according to 2022 Controller Mapping document on 2/12/2022 

    public static final int ENABLE_LAUNCHER_BUTTON  = 5; // Left bumper
    public static final int DISABLE_LAUNCHER_BUTTON = 6; // Right bumper

    // Joystick Ports
    public static final int OI_DRIVER_LEFT_JOYSTICK  = 0;
    public static final int OI_DRIVER_RIGHT_JOYSTICK = 1;
    public static final int OI_OPERATOR_CONTROLLER   = 2;

    // Controls set for Arcade Drive - left stick turn, right stick throttle.
    public static final int OI_DRIVER_TURN = 2;  // Z Axis for Extreme 3D Pro
    public static final int OI_DRIVER_MOVE = 1;  // Y Axis for Extreme 3D Pro

    // Controls set for Tank Drive
    public static final int OI_DRIVER_SPEED_AXIS = 1;

    // Driver Controls
    public static final int OI_DRIVER_SHOOT          = 2;  // Joystick Button 2
    public static final int OI_DRIVER_SET_LOW_GEAR   = 3;  // Joystick Button 3
    public static final int OI_DRIVER_SET_HIGH_GEAR  = 5;  // Joystick Button 5
    public static final int OI_DRIVER_SLOWMODE       = 4;  // Joystick Button 4
    public static final int OI_DRIVER_ACQUIRE_TARGET = 1;  // Joystick Trigger Button
    public static final int OI_DRIVER_REVERSE        = 0;  // Joystick POV number

    public static final int OI_OPERATOR_TRANSFER_EJECT    = 4;  // Y button
    public static final int OI_OPERATOR_TRANSFER_LOAD     = 3;  // X button
    public static final int OI_OPERATOR_INTAKE_EXTEND     = 1;  // A Button
    public static final int OI_OPERATOR_INTAKE_RETRACT    = 2;  // B Button
    public static final int OI_OPERATOR_EXTEND_CLIMB      = 3;  // Right Trigger Axis
    public static final int OI_OPERATOR_RETRACT_CLIMB     = 6;  // Right Bumper
    public static final int OI_OPERATOR_ORCHESTRATE_CLIMB = 5;  // Left Bumper
    public static final int OI_OPERATOR_STOP_CLIMB        = 2;  // Left Trigger Axis (TODO: Write this command)

    public static final Port i2c = Port.kOnboard;
}
