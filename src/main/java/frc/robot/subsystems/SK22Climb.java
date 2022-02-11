package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.subsystems.base.MotorEncoder;

public class SK22Climb extends SKSubsystemBase
{

    private final SK22ComplexClimbArm leftBackArm;
    private final SK22ComplexClimbArm rightBackArm;
    private final SK22SimpleClimbArm leftFrontArm;
    private final SK22SimpleClimbArm rightFrontArm;

    public SK22Climb(CANSparkMax complexBrakePivot, CANSparkMax complexRatchetLift, MotorEncoder climbEncoder)
    {
        leftBackArm = new SK22ComplexClimbArm(complexBrakePivot, complexRatchetLift, climbEncoder);
        rightBackArm = new SK22ComplexClimbArm(complexBrakePivot, complexRatchetLift, climbEncoder);
        leftFrontArm = new SK22SimpleClimbArm();
        rightFrontArm = new SK22SimpleClimbArm();

    }

    public void extendLeftArm(int speed, int distance)
    {
        // Releases the piston to extend the arm outward or have the hand in the proper position to hook
        leftFrontArm.raise();
        leftBackArm.turnComplexRatchetLiftOn(speed);
        if(leftBackArm.getBackMotorPosition() == distance)
        {
            leftBackArm.turnComplexRatchetLiftOff();
        }
    }

    public void retract()
    {
        // Pulls back the piston to retract the arm or have the hand pulled back in order to prevent crushing it while pulling up
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
