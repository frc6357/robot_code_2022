package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.subsystems.base.ComplexClimbArm;
/**
 * Subsystem for the complex arms
 */
public class SK22ComplexClimb extends SubsystemBase
{

    private final ComplexClimbArm complexArm;

    /**
     * Constructor initalizes the complexbrakes, ratches and encoders
     */
    public SK22ComplexClimb()
    {
        final CANSparkMax complexBrakePivot = new CANSparkMax(Ports.COMPLEX_BRAKE_PIVOT, MotorType.kBrushless);
        final CANSparkMax complexRatchetLift = new CANSparkMax(Ports.COMPLEX_RATCHET_LIFT, MotorType.kBrushless);
        final RelativeEncoder pivotEncoder = complexBrakePivot.getEncoder();
        final RelativeEncoder ratchetEncoder = complexRatchetLift.getEncoder();

        complexArm = new ComplexClimbArm(complexBrakePivot, complexRatchetLift, pivotEncoder, ratchetEncoder);
    }

    /**
     * Extends the complex arms
     */
    public void raiseComplexArm()
    {
        complexArm.raise();
    }

    /**
     * Retracts the complex arms
     */
    public void lowerComplexArm()
    {
        complexArm.lower();
    }

    /**
     * Rotates the complex arms either upwards or downwards
     * @param direction the way in which the complex arms turn (true = upwards) (false = downwards)
     */
    public void rotateComplexArm(boolean direction)
    {
        complexArm.rotate(direction);
    }

     /**
     * sets the angle position of the complex arm
     * @param degrees The position we want the arm to pivot to.
     */
    public void setComplexPivotPosition(double degrees)
    {
        complexArm.setPivotPosition(degrees);
    }

    /**
     * sets the extension position of the complex arm
     * @param distance The position we want the arm to extend to.
     */
    public void setComplexRatchetArmPosition(double distance)
	{
        complexArm.setRatchetArmPosition(distance);
	}

    /**
     * gets voltage produced by the ratchet arm
     * @return the voltage used by the ratchet motor
     */
    public double getMotorCurrent()
    {
        return complexArm.getCurrent();
    }

    
}
