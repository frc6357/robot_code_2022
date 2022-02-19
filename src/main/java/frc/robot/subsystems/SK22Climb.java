package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Ports;
import frc.robot.subsystems.base.ComplexClimbArm;
import frc.robot.subsystems.base.SimpleClimbArm;

/**
 * A subsystem that characterizes climb
 */
public class SK22Climb extends SKSubsystemBase
{

    private final ComplexClimbArm complexArm;
    private final SimpleClimbArm simpleArm;


    // and to allow us to get the code to build after a breaking commit.
    /**
     *  The climb constructor initializes the variables
     */
    public SK22Climb()
    {
        final CANSparkMax complexBrakePivot = new CANSparkMax(Ports.COMPLEX_BRAKE_PIVOT, MotorType.kBrushless);
        final CANSparkMax complexRatchetLift = new CANSparkMax(Ports.COMPLEX_RATCHET_LIFT, MotorType.kBrushless);
        final RelativeEncoder pivotEncoder = complexBrakePivot.getEncoder();
        final RelativeEncoder ratchetEncoder = complexRatchetLift.getEncoder();

        complexArm = new ComplexClimbArm(complexBrakePivot, complexRatchetLift, pivotEncoder, ratchetEncoder);
        simpleArm = new SimpleClimbArm();
    }

    /**
     * extends the simple arm
     */
    public void raiseSimpleArm()
    {
        simpleArm.raise();
        // Extends arm from the simpleArm base class
    }

    /**
     * retracts the simple arm
     */
    public void lowerSimpleArm()
    {
        simpleArm.lower();
        // Pulls back the piston to retract the arm or have the hand pulled back in order 
        //to prevent crushing it while pulling up
    }

    /**
     * sets the position of the complex arm
     * @param degrees The position we want the arm to pivot to.
     */
    public void setComplexArm(double degrees)
    {
        complexArm.setPivotArmPosition(degrees);
    }

 
    /**
     * rotates simple arm so that it is perpendicular with the robot
     */
    public void straightenSimpleArm()
    {
        simpleArm.straighten();
        // Straighten the simpleclimbarm
    }

    /**
     * Moves the the simple arm back into a non perpendicular state
     */
    public void tiltSimpleArm()
    {
        simpleArm.tilt();
    }

    /**
     * gets voltage produced by the ratchet arm
     * @return the voltage used by the ratchet motor
     */
    public double getMotorCurrent()
    {
        return complexArm.getVoltage();
    }

    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public void initializeTestMode()
    {

    }

    @Override
    public void testModePeriodic()
    {

    }

    @Override
    public void enterTestMode()
    {

    }
}
