package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Ports;

public class SimpleClimbArm
{
    private final DoubleSolenoid simpleLiftPiston = new DoubleSolenoid(
        Ports.CLIMB_PNEUMATIC_MODULE, PneumaticsModuleType.REVPH,
        Ports.SIMPLE_CLIMB_LIFT_PISTON_FORWARD_CHANNEL, Ports.SIMPLE_CLIMB_LIFT_PISTON_REVERSE_CHANNEL);

    private final DoubleSolenoid simpleTiltPiston = new DoubleSolenoid(Ports.CLIMB_PNEUMATIC_MODULE,
        PneumaticsModuleType.REVPH, Ports.SIMPLE_CLIMB_TILT_PISTON_FORWARD_CHANNEL,
        Ports.SIMPLE_CLIMB_TILT_PISTON_REVERSE_CHANNEL); 

    public SimpleClimbArm()
    {

    }

    public void raise()
    {
        simpleLiftPiston.set(DoubleSolenoid.Value.kForward);
    }

    public void lower()
    {
        simpleLiftPiston.set(DoubleSolenoid.Value.kReverse);
    }

    public void tilt()
    {
        simpleTiltPiston.set(DoubleSolenoid.Value.kForward);
    }

    public void straighten()
    {
        simpleTiltPiston.set(DoubleSolenoid.Value.kReverse);
    }
}
