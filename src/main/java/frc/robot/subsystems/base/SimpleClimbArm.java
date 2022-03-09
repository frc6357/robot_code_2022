package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Ports;

/**
 * A class to control the simple, pneumatically-actuated arm in the climb mechanism of the
 * 2022 robot.
 */
public class SimpleClimbArm
{
    // TODO: Hardware no longer supports the tilt function. Remove this from the class.
    private final Solenoid raisePiston = new Solenoid(Ports.CLIMB_PNEUMATIC_MODULE,
        PneumaticsModuleType.REVPH, Ports.SIMPLE_CLIMB_RAISE_CHANNEL);
        private final Solenoid lowerPiston = new Solenoid(Ports.CLIMB_PNEUMATIC_MODULE,
        PneumaticsModuleType.REVPH, Ports.SIMPLE_CLIMB_LOWER_CHANNEL);

    private final Solenoid tiltPiston = new Solenoid(Ports.CLIMB_PNEUMATIC_MODULE,
        PneumaticsModuleType.REVPH, Ports.SIMPLE_CLIMB_TILT_CHANNEL);
    private final Solenoid straightenPiston = new Solenoid(Ports.CLIMB_PNEUMATIC_MODULE,
        PneumaticsModuleType.REVPH, Ports.SIMPLE_CLIMB_STRAIGHTEN_CHANNEL);

    
    /**
     * Constructor for the SimpleClimbArm
     */
    public SimpleClimbArm()
    {

    }

    /**
     * Activate the solenoid required to raise the arm.
     */
    public void raise()
    {
        raisePiston.set(true);
        lowerPiston.set(false);
    }

    /**
     * Activate the solenoid required to lower the arm.
     */
    public void lower()
    {
        lowerPiston.set(true);
        raisePiston.set(false);
    }

    /**
     * Neutral mode for lift Solenoids
     */
    public void liftNeutral()
    {
        raisePiston.set(false);
        lowerPiston.set(false);
    }
    /**
     * Activate the solenoid required to tilt the arm away from the vertical position.
     */
    public void tilt()
    {
        tiltPiston.set(true);
        straightenPiston.set(false);
    }

    /**
     * Activate the solenoid required to return the arm to the vertical position.
     */
    public void straighten()
    {
        straightenPiston.set(true);
        tiltPiston.set(false);
    }

    /**
     * Neutral mode for tilt Solenoids
     */
    public void tiltNeutral()
    {
        straightenPiston.set(false);
        tiltPiston.set(false);
    }
}
