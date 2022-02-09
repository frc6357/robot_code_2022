package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Ports;
import frc.robot.Constants.ClimbConstants;

public class SK22SimpleClimbArm
{
    private final DoubleSolenoid handPiston = new DoubleSolenoid(Ports.ClimbPneumaticModule,
        PneumaticsModuleType.REVPH, Ports.ClimbHandForwardChannel, Ports.ClimbHandReverseChannel);

    private final DoubleSolenoid simpleLiftPiston = new DoubleSolenoid(Ports.ClimbPneumaticModule,
        PneumaticsModuleType.REVPH, Ports.SimpleClimbLiftPistonForwardChannel,
        Ports.SimpleClimbLiftPistonReverseChannel);

    private final DoubleSolenoid simpleTiltPiston = new DoubleSolenoid(Ports.ClimbPneumaticModule,
        PneumaticsModuleType.REVPH, Ports.SimpleClimbTiltPistonForwardChannel,
        Ports.SimpleClimbTiltPistonReverseChannel);

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
