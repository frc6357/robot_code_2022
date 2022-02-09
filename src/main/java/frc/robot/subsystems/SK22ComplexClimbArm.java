package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Ports;

/**
 * SK22ComplexClimbArm is meant to serve as the robot's front climb arm in which it will reach for the high rung
 * and the traversl rung as well.
 */
public class SK22ComplexClimbArm
{
    private final CANSparkMax complexBrakePivot;
    private final CANSparkMax complexRatchetLift;

    private final DoubleSolenoid handPiston = new DoubleSolenoid(Ports.ClimbPneumaticModule,
        PneumaticsModuleType.REVPH, Ports.ClimbHandForwardChannel, Ports.ClimbHandReverseChannel);
    private final DoubleSolenoid complexBrakePiston =
            new DoubleSolenoid(Ports.ClimbPneumaticModule, PneumaticsModuleType.REVPH,
                Ports.ClimbBrakePistonForwardChannel, Ports.ClimbBrakePistonReverseChannel);
    private final DoubleSolenoid complexRatchetPiston =
            new DoubleSolenoid(Ports.ClimbPneumaticModule, PneumaticsModuleType.REVPH,
                Ports.ComplexClimbRatchetPistonForwardChannel,
                Ports.ComplexClimbRatchetPistonReverseChannel);

    /**
     * The constructor requires the two CANSparkMax motors for rotation of the arm and the extending vertically
     * of the arm.
     * @param complexBrakePivot The motor used for the rotation of the arm
     * @param complexRatchetLift The motor used for the extention of the arm vertically
     */
    public SK22ComplexClimbArm(CANSparkMax complexBrakePivot, CANSparkMax complexRatchetLift)
    {
        this.complexBrakePivot = complexBrakePivot;
        this.complexRatchetLift = complexRatchetLift;
    }

    /**
     * The retract hand method will turn off the piston in the hand so that we can lift the robot onto the
     * middle rung without crushing the hand
     */
    public void retractHand()
    {
        handPiston.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * The extend hand method will turn on the piston in the hand so that we can extend and grab the
     * high rung
     */
    public void extendHand()
    {
        handPiston.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * The turnBrakePivotOn method will turn on the motor that will allow the front hand
     * to rotate in order to grab onto the high and traversal rung
     * @param speed The speed will change how fast the motor rotates to a specific angle
     */
    public void turnBrakePivotOn(double speed)
    {
        complexBrakePivot.set(speed);

    }

    /**
     * The turnBrakePivotOff method will turn off the motor that's controlling the
     * rotation of the arm so that we don't over-rotate
     */
    public void turnBrakePivotOff()
    {
        complexBrakePivot.stopMotor();
    }

    /**
     * The turnBrakePistonOn method will turn on the piston in the rotational portion of
     * the arm so that the arm can properly stop at a specific angle
     */
    public void turnBrakePistonOn()
    {
        complexBrakePiston.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * The turnBrakePistonOff method will turn off the piston in the rotational portion of
     * the arm so that the motor can rotate to a different position
     */
    public void turnBrakePistonOff()
    {
        complexBrakePiston.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * The turnComplexRatchetLiftOn method will turn on the motor that is used to extend the arm via gears
     * @param speed The speed will determine how fast the motor will extend the arm
     */
    public void turnComplexRatchetLiftOn(int speed)
    {
        complexRatchetLift.set(speed);
    }

    /**
     * The turnComplexRatchetLiftOff method will turn off the motor so that the arm will stop after a certain
     * point once extended
     */
    public void turnComplexRatchetLiftOff()
    {
        complexRatchetLift.stopMotor();
    }

    /**
     * The turnRatchetPistonOn method will turn on the piston to extend the hand in order to grab a rung
     */
    public void turnRatchetPistonOn()
    {
        complexRatchetPiston.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * The turnRatchetPistonOn method will turn off the piston to stop the extension of the hand to prevent
     * over extending
     */
    public void turnRatchetPistonOff()
    {
        complexRatchetPiston.set(DoubleSolenoid.Value.kReverse);
    }
}
