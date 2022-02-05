package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Ports;
import frc.robot.simulation.RobotSim;

public class SK22Climb extends SKSubsystemBase
{
    private final CANSparkMax complexBrakePivot = new CANSparkMax(Ports.ComplexBrakePivot, MotorType.kBrushless);
    private final CANSparkMax complexRatchetLift = new CANSparkMax(Ports.ComplexRatchetLift, MotorType.kBrushless);

    private final SK22ComplexClimbArm backArm = new SK22ComplexClimbArm(complexBrakePivot, complexRatchetLift);

    private final RobotSim simulation = new RobotSim(complexBrakePivot, complexRatchetLift);

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
        /*Performs the motion of switichuing the rotation of the arm while retracting the arm to essentially have a linear
        motion. While this is happening we need to extent the back arm outward */
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