package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.base.SimpleClimbArm;

/**
 * This subsystem controls the complex climb arm
 */
public class SK22SimpleClimb extends SubsystemBase
{
    private final SimpleClimbArm simpleArm;

    /**
     * constructor initalizes the simple arm base class
     */
    public SK22SimpleClimb()
    {
        simpleArm = new SimpleClimbArm();
    }

    /**
     * Raises the simple arm
     */
    public void raiseSimpleArm()
    {
        simpleArm.raise();
        // Extends arm from the simpleArm base class
    }

    /**
     * retracts the simple arm
     */
    public void lowerSimpleArm()
    {
        simpleArm.lower();
        // Pulls back the piston to retract the arm or have the hand pulled back in order 
        //to prevent crushing it while pulling up
    }

    /**
     * Makes the lift arm command go into a neutral position
     */
    public void makeRaiseSimpleArmNeutral()
    {
        simpleArm.neutral();
    }

    
}
