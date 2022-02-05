package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Ports;

public class SK22ComplexClimbArm
{
    private final CANSparkMax complexBrakePivot;
    private final CANSparkMax complexRatchetLift;

    private final DoubleSolenoid handPiston = new DoubleSolenoid(Ports.ClimbPneumaticModule, PneumaticsModuleType.REVPH,
                                                                 Ports.ClimbHandForwardChannel, Ports.ClimbHandReverseChannel);
    private final DoubleSolenoid complexBrakePiston = new DoubleSolenoid(Ports.ClimbPneumaticModule, PneumaticsModuleType.REVPH,
                                                                         Ports.ClimbBrakePistonForwardChannel, Ports.ClimbBrakePistonReverseChannel);
    private final DoubleSolenoid complexRatchetPiston = new DoubleSolenoid(Ports.ClimbPneumaticModule, PneumaticsModuleType.REVPH, 
                                                                           Ports.ComplexClimbRatchetPistonForwardChannel, Ports.ComplexClimbRatchetPistonReverseChannel);

    public SK22ComplexClimbArm(CANSparkMax complexBrakePivot, CANSparkMax complexRatchetLift)
    {
        this.complexBrakePivot = complexBrakePivot;
        this.complexRatchetLift = complexRatchetLift;
    }

    public void retractArm()
    {
        handPiston.set(DoubleSolenoid.Value.kReverse);
    }

    public void extendArm()
    {
        handPiston.set(DoubleSolenoid.Value.kForward);
    }

    public void turnBrakePivotOn(double speed)
    {
        complexBrakePivot.set(speed);

    }

    public void turnBrakePivotOff()
    {
        complexBrakePivot.stopMotor();
    }

    public void turnBrakePistonOn()
    {
        complexBrakePiston.set(DoubleSolenoid.Value.kForward);
    }

    public void turnBrakePistonOff()
    {
        complexBrakePiston.set(DoubleSolenoid.Value.kReverse);
    }
    
    public void turnComplexRatchetLiftOn(int speed)
    {
        complexRatchetLift.set(speed);
    }

    public void turnComplexRatchetLiftOff()
    {
        complexRatchetLift.stopMotor();
    }

    public void turnRatchetPistonOn()
    {
        complexRatchetPiston.set(DoubleSolenoid.Value.kForward);
    }
    
    public void turnRatchetPistonOff()
    {
        complexRatchetPiston.set(DoubleSolenoid.Value.kReverse);
    }
}
