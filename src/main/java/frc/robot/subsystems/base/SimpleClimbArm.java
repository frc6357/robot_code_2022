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
    private final Solenoid raisePiston = new Solenoid(Ports.CLIMB_PNEUMATIC_MODULE,
        PneumaticsModuleType.REVPH, Ports.SIMPLE_CLIMB_RAISE_CHANNEL);
        private final Solenoid lowerPiston = new Solenoid(Ports.CLIMB_PNEUMATIC_MODULE,
        PneumaticsModuleType.REVPH, Ports.SIMPLE_CLIMB_LOWER_CHANNEL);

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
    public void neutral()
    {
        raisePiston.set(false);
        lowerPiston.set(false);
    }
}
