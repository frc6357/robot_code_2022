package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Ports;
import frc.robot.Constants.ClimbConstants;

public class SK22SimpleClimbArm
{
    private final CANSparkMax simpleRatchetLift =
            new CANSparkMax(Ports.SIMPLE_RATCHET_LIFT, ClimbConstants.MOTOR_TYPE);

    private final DoubleSolenoid handPiston = new DoubleSolenoid(Ports.CLIMB_PNEUMATIC_MODULE,
        PneumaticsModuleType.REVPH, Ports.CLIMB_HAND_FORWARD_CHANNEL, Ports.CLIMB_HAND_REVERSE_CHANNEL);

    private final DoubleSolenoid simpleRatchetPiston = new DoubleSolenoid(
        Ports.CLIMB_PNEUMATIC_MODULE, PneumaticsModuleType.REVPH,
        Ports.SIMPLE_CLIMB_RATCHET_PISTON_FORWARD_CHANNEL, Ports.SIMPLE_CLIMB_RATCHET_PISTON_REVERSE_CHANNEL);

    public SK22SimpleClimbArm(CANSparkMax simpleRatchetLift)
    {

    }

    public void retractArm()
    {
        handPiston.set(DoubleSolenoid.Value.kReverse);
    }

    public void extendArm()
    {
        handPiston.set(DoubleSolenoid.Value.kForward);
    }

    public void turnComplexRatchetLiftOn(int speed)
    {
        simpleRatchetLift.set(speed);
    }

    public void turnComplexRatchetLiftOff()
    {
        simpleRatchetLift.stopMotor();
    }

    public void turnRatchetPistonOn()
    {
        simpleRatchetPiston.set(DoubleSolenoid.Value.kForward);
    }

    public void turnRatchetPistonOff()
    {
        simpleRatchetPiston.set(DoubleSolenoid.Value.kReverse);
    }
}
