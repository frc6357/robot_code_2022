// 
// Definitions of addresses and hardware port numbers used in
// the FRC Team 6357 2022 robot.
//
package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

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
    public static final int FRONT_LEFT_DRIVE  = 10; // CAN ID
    public static final int FRONT_RIGHT_DRIVE = 11; // CAN ID
    public static final int BACK_LEFT_DRIVE   = 12; // CAN ID
    public static final int BACK_RIGHT_DRIVE  = 13; // CAN ID

    ///////////////////////////////
    // Additional hardware
    ///////////////////////////////
    public static final int MindSensorsCANLight = 3;  // CAN ID of LED strip controller
}
