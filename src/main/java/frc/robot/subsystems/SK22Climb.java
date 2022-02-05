package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Ports;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.subsystems.SK22ComplexClimbArm;



public class SK22Climb extends SKSubsystemBase
{

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