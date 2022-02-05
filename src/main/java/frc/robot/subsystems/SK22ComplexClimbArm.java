package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Ports;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class SK22ComplexClimbArm
{
    private final CANSparkMax complexBrakePivot = new CANSparkMax(Ports.ComplexBrakePivot, MotorType.kBrushless);
    private final CANSparkMax complexRatchetLift = new CANSparkMax(Ports.ComplexRatchetLift, MotorType.kBrushless);
    private final DoubleSolenoid handPiston = new DoubleSolenoid(Constants.ClimbConstants.PneumaticModule, PneumaticsModuleType.REVPH, Constants.ClimbConstants.ForwardChannel, Constants.ClimbConstants.ReverseChannel);
    private final DoubleSolenoid complexBrakePiston = new DoubleSolenoid(Constants.ClimbConstants.PneumaticModule, PneumaticsModuleType.REVPH, Constants.ClimbConstants.ForwardChannel, Constants.ClimbConstants.ReverseChannel);
    private final DoubleSolenoid complexRatchetPiston = new DoubleSolenoid(Constants.ClimbConstants.PneumaticModule, PneumaticsModuleType.REVPH, Constants.ClimbConstants.ForwardChannel, Constants.ClimbConstants.ReverseChannel);

    public SK22ComplexClimbArm()
    {
        
    }

    public void retractArm()
    {
        handPiston.set(DoubleSolenoid.Value.kReverse);
    }

    public void tilt()
    {
        
    }
}
