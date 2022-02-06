package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

public class SK22Climb extends SKSubsystemBase
{

    private final SK22ComplexClimbArm backArm;

    public SK22Climb(CANSparkMax complexBrakePivot, CANSparkMax complexRatchetLift)
    {
        backArm = new SK22ComplexClimbArm(complexBrakePivot, complexRatchetLift);
    }

    public void extend()
    {
        // Releases the piston to extend the arm outward or have the hand in the proper position to hook
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
