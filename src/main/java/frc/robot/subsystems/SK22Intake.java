//
//SK22Intake: This is the system for getting cargo into the robot in the 2022 season.
//

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The SK22Intake class is the subsytem that controls the intake of the Robot.
 */
public class SK22Intake extends SKSubsystemBase
{
    // TODO: Remove commenting on pneumatics!
    private final DoubleSolenoid intakeMover = new DoubleSolenoid(Ports.BASE_PCM, Ports.INTAKE_PCM,
        Ports.INTAKE_MOVER_FORWARD, Ports.INTAKE_MOVER_BACKWARD);

    private final CANSparkMax intakeMotor = new CANSparkMax(Ports.INTAKE_MOTOR, MotorType.kBrushless);

    /** Creates a new SK22Intake. */
    public SK22Intake()
    {
        SmartDashboard.putString("Intake Extended", "IN");
        intakeMotor.setInverted(true);
    
    }

    /**
     * Disables the motors of the and turns the double solenoid off
     */
    public void disableIntake()
    {
        intakeMotor.set(0.0);
        intakeMover.set(DoubleSolenoid.Value.kOff);
    }

    /**
     * Extends the intake position out of the robot
     */
    public void extendIntake()
    {
        intakeMover.set(DoubleSolenoid.Value.kForward);
        SmartDashboard.putString("Intake Extended", "OUT");
    }

    /**
     * Retracts the intake position into the robot
     */
    public void retractIntake()
    {
        intakeMover.set(DoubleSolenoid.Value.kReverse);
        SmartDashboard.putString("Intake Extended", "IN");
    }

    /**
     * Sets the speed of the motor of the intake that is intended to pick up balls
     * 
     * @param speed A value from -1.0 to 1.0
     */
    public void setIntakeSpeed(double speed)
    {
        intakeMotor.set(speed);
        SmartDashboard.putNumber("Intake Speed", speed);
    }

    /**
     * Returns the current speed of the intake motor
     * 
     * @return A value between -1.0 and 1.0
     */
    public double getIntakeSpeed()
    {
        // return intakeMotor.get();
        return 0.0;
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
