package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Ports;
import frc.robot.subsystems.base.ComplexClimbArm;
import frc.robot.subsystems.base.MotorEncoder;
import frc.robot.subsystems.base.SimpleClimbArm;

public class SK22Climb extends SKSubsystemBase
{

    private final ComplexClimbArm leftBackArm;
    private final ComplexClimbArm rightBackArm;
    private final SimpleClimbArm leftFrontArm;
    private final SimpleClimbArm rightFrontArm;

    private final SimpleClimbArm simpleClimbArm;
    private final ComplexClimbArm complexClimbArm;

    // TODO: Updated to conform to convention of not passing objects into subsystem constructors
    // and to allow us to get the code to build after a breaking commit.
    public SK22Climb()
    {
        // TODO: Verify that these are correct
        final CANSparkMax complexBrakePivot = new CANSparkMax(Ports.COMPLEX_BRAKE_PIVOT, MotorType.kBrushless);
        final CANSparkMax complexRatchetLift  = new CANSparkMax(Ports.COMPLEX_RATCHET_LIFT, MotorType.kBrushless);;
        final RelativeEncoder climbEncoder = complexBrakePivot.getEncoder();

        leftBackArm = new ComplexClimbArm(complexBrakePivot, complexRatchetLift, climbEncoder);
        rightBackArm = new ComplexClimbArm(complexBrakePivot, complexRatchetLift, climbEncoder);
        leftFrontArm = new SimpleClimbArm();
        rightFrontArm = new SimpleClimbArm();

        simpleClimbArm = new SimpleClimbArm();
        complexClimbArm = new ComplexClimbArm(complexBrakePivot, complexRatchetLift, climbEncoder);
    }

    public void raise()
    {
        simpleClimbArm.raise();
        // Extends arm from the simpleArm base class
    }

    public void lower()
    {
        simpleClimbArm.lower();
        // Pulls back the piston to retract the arm or have the hand pulled back in order to prevent crushing it while pulling up
    }

    public void straighten(){
        simpleClimbArm.straighten();
        // Straighten the simpleclimbarm
    }

    public void tilt(){
        simpleClimbArm.tilt();
        // tilt the simpleclimbarm
    }

    

    public void orchestra()
    {
        /*
         * Performs the motion of switichuing the rotation of the arm while retracting the
         * arm to essentially have a linear motion. While this is happening we need to
         * extent the back arm outward
         */
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
