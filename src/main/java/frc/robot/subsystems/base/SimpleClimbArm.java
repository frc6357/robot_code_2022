package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Ports;

/**
 * A class to control the simple, pneumatically-actuated arm in the climb mechanism of the
 * 2022 robot.
 */
public class SimpleClimbArm
{
    private final DoubleSolenoid simpleLiftPiston = new DoubleSolenoid(Ports.CLIMB_PNEUMATIC_MODULE,
        PneumaticsModuleType.REVPH, Ports.SIMPLE_CLIMB_LIFT_PISTON_FORWARD_CHANNEL,
        Ports.SIMPLE_CLIMB_LIFT_PISTON_REVERSE_CHANNEL);

    private final DoubleSolenoid simpleTiltPiston = new DoubleSolenoid(Ports.CLIMB_PNEUMATIC_MODULE,
        PneumaticsModuleType.REVPH, Ports.SIMPLE_CLIMB_TILT_PISTON_FORWARD_CHANNEL,
        Ports.SIMPLE_CLIMB_TILT_PISTON_REVERSE_CHANNEL);

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
        simpleLiftPiston.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Activate the solenoid required to lower the arm.
     */
    public void lower()
    {
        simpleLiftPiston.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Activate the solenoid required to tilt the arm away from the vertical position.
     */
    public void tilt()
    {
        simpleTiltPiston.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Activate the solenoid required to return the arm to the vertical position.
     */
    public void straighten()
    {
        simpleTiltPiston.set(DoubleSolenoid.Value.kReverse);
    }
}
