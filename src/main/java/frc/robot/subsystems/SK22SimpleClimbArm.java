package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Ports;
import frc.robot.Constants.ClimbConstants;

public class SK22SimpleClimbArm
{
    private final CANSparkMax simpleRatchetLift = new CANSparkMax(Ports.SimpleRatchetLift, ClimbConstants.MOTOR_TYPE);

    private final DoubleSolenoid handPiston = new DoubleSolenoid(Ports.ClimbPneumaticModule, PneumaticsModuleType.REVPH,
                                                                 Ports.ClimbHandForwardChannel, Ports.ClimbHandReverseChannel);

    private final DoubleSolenoid simpleRatchetPiston = new DoubleSolenoid(Ports.ClimbPneumaticModule, PneumaticsModuleType.REVPH, 
                                                                           Ports.SimpleClimbRatchetPistonForwardChannel, Ports.SimpleClimbRatchetPistonReverseChannel);

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