// 
// Definitions of addresses and hardware port numbers used in
// the FRC Team 6357 2022 robot.
//
package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

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
    public static final int BASE_PCM                    = 1;  // CAN ID
    public static final int ClimbPneumaticModule        = 2;

    public static final PneumaticsModuleType pneumaticsModuleType = PneumaticsModuleType.REVPH;

    //////////////////////////////
    // I2C device addresses
    //////////////////////////////
    public static final Port i2cColorSensor             =  I2C.Port.kOnboard;

    //////////////////////////////
    // Drivetrain Addresses
    //////////////////////////////
    public static final int frontLeftDrive              = 10; // CAN ID
    public static final int frontRightDrive             = 11; // CAN ID
    public static final int backLeftDrive               = 12; // CAN ID
    public static final int backRightDrive              = 13; // CAN ID

    public static final int gearShiftHigh               = 2;
    public static final int gearShiftLow                = 3;

    //////////////////////////////
    // Transfer Addresses
    //////////////////////////////
    public static final int exitTransferMotor           = 12; // Currently unknown Port
    public static final int intakeTransferMotor         = 12; // Currently unknown Port
    public static final int verticalTransferMotor       = 12; // Currently unknown Port

    //////////////////////////////
    // Intake Addresses
    //////////////////////////////
    public static final int intakeMotor                 = 31; // CAN ID 

    public static final int intakeMoverForward          = 0;  // PCM Channel 
    public static final int intakeMoverBackward         = 1;  // PCM Channel 

    public static final PneumaticsModuleType intakePCM  = PneumaticsModuleType.CTREPCM;


    ///////////////////////////////
    // Ball Indexer Addresses
    ///////////////////////////////
    

    ///////////////////////////////
    // Ball Launcher Addresses
    ///////////////////////////////
    public static final int ballLauncher1              = 23; // CAN ID
    public static final int ballLauncher2              = 24; // CAN ID

    ///////////////////////////////
    // Climb Addresses
    ///////////////////////////////
    public static final int ComplexBrakePivot                       = 1; // Currently unknown Port
    public static final int ComplexRatchetLift                      = 2; // Currently unknown Port
    public static final int SimpleRatchetLift                       = 3; // Currently unkown Port
    public static final int ClimbHandForwardChannel                 = 4; 
    public static final int ClimbHandReverseChannel                 = 5;
    public static final int ClimbBrakePistonForwardChannel          = 6; 
    public static final int ClimbBrakePistonReverseChannel          = 7;
    public static final int ComplexClimbRatchetPistonForwardChannel        = 8; 
    public static final int ComplexClimbRatchetPistonReverseChannel        = 9;
    public static final int SimpleClimbRatchetPistonForwardChannel        = 10; 
    public static final int SimpleClimbRatchetPistonReverseChannel        = 11;

    ///////////////////////////////
    // Additional hardware
    ///////////////////////////////
    public static final int MindSensorsCANLight         = 3;  // CAN ID of LED strip controller

    ///////////////////////////////
    // Operator Interface
    ///////////////////////////////
    public static final int ENABLE_LAUNCHER_BUTTON        = 5; // Left bumper
    public static final int DISABLE_LAUNCHER_BUTTON       = 6; // Right bumper
    
    // Joystick Ports
    public static final int OIDriverLeftJoystick        = 0;
    public static final int OIDriverRightJoystick       = 1;
    public static final int OIOperatorController        = 2;

    // Controls set for Arcade Drive - left stick turn, right stick throttle.
    public static final int OIDriverTurn                = 2;  // Z Axis for Extreme 3D Pro
    public static final int OIDriverMove                = 1;  // Y Axis for Extreme 3D Pro

    // Controls set for Tank Drive
    public static final int OIDriverSpeedAxis           = 1;               
    
    // Driver Controls
    public static final int OIDriverShoot               = 2;  // Joystick Button 2s
    public static final int OIDriverSetLowGear          = 3;  // Joystick Button 3
    public static final int OIDriverSetHighGear         = 4;  // Joystick Button 4
    public static final int OIDriverSlowmode            = 5;  // Joystick Button 5
    public static final int OIDriverAcquireTarget       = 1;  // Joystick Trigger Button

    // TODO: Would need to choose the correct button and ID.
    public static final int OIOperatorEjectBallButton   = 4;  // Y button
    public static final int OIOperatorIntakeExtend      = 1;  // A Button
    public static final int OIOperatorIntakeRetract     = 2;  // B Button
    public static final int OIOperatorExtendClimb       = 3;  // Right Trigger Axis
    public static final int OIOperatorRetractClimb      = 6;  // Right Bumper
    public static final int OIOperatorOrchestrateClimb  = 5;  // Left Bumper

    public static Port i2c;

}
