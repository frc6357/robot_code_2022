package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Ports;
import frc.robot.Constants.ClimbConstants;

public class SK22SimpleClimbArm
{
    private final DoubleSolenoid handPiston = new DoubleSolenoid(Ports.CLIMB_PNEUMATIC_MODULE,
        PneumaticsModuleType.REVPH, Ports.CLIMB_HAND_FORWARD_CHANNEL, Ports.CLIMB_HAND_REVERSE_CHANNEL);

    private final DoubleSolenoid simpleLiftPiston = new DoubleSolenoid(
        Ports.CLIMB_PNEUMATIC_MODULE, PneumaticsModuleType.REVPH,
        Ports.SIMPLE_CLIMB_RATCHET_PISTON_FORWARD_CHANNEL, Ports.SIMPLE_CLIMB_RATCHET_PISTON_REVERSE_CHANNEL);

    private final DoubleSolenoid simpleTiltPiston = new DoubleSolenoid(Ports.CLIMB_PNEUMATIC_MODULE,
        PneumaticsModuleType.REVPH, Ports.SIMPLE_CLIMB_RATCHET_PISTON_FORWARD_CHANNEL,
        Ports.SIMPLE_CLIMB_RATCHET_PISTON_REVERSE_CHANNEL); //TODO: make sure this is the right ports

    public SK22SimpleClimbArm()
    {

    }

    public void retractHand()
    {
        handPiston.set(DoubleSolenoid.Value.kReverse);
    }

    public void extendHand()
    {
        handPiston.set(DoubleSolenoid.Value.kForward);
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
