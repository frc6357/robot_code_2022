//
//SK22Intake: This is the system for getting cargo into the robot in the 2022 season.
//

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Ports;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SK22Intake extends SKSubsystemBase
{
    private final DoubleSolenoid intakeMover = new DoubleSolenoid(Ports.INTAKE_PCM,
        Ports.INTAKE_MOVER_FORWARD, Ports.INTAKE_MOVER_BACKWARD);

    private final CANSparkMax intakeMotor = new CANSparkMax(Ports.INTAKE_MOTOR, MotorType.kBrushless);
    private double motorSpeed = 0.0;

    /** Creates a new SK22Intake. */
    public SK22Intake()
    {
        
    }

    // This should be the safe and initial code that happens. nothing should move
    public void disableIntake()
    {
        motorSpeed = 0.0;
        intakeMotor.set(motorSpeed);
        intakeMover.set(DoubleSolenoid.Value.kOff);
    }

    // This will extend the double solenoids, moving the intake to the active
    // position
    public void extendIntake()
    {
        intakeMover.set(DoubleSolenoid.Value.kForward);
        SmartDashboard.putBoolean("Intake Extended", true);
    }

    // This will retract the double solenoids, moving the intake to the inactive
    // position
    public void retractIntake()
    {
        intakeMover.set(DoubleSolenoid.Value.kReverse);
        SmartDashboard.putBoolean("Intake Extended", false);
    }

    // This will set the speed of the motor that powers the ball pulling part of the
    // intake
    public void setIntakeSpeed(double speed)
    {
        motorSpeed = speed;
        intakeMotor.set(motorSpeed);
        SmartDashboard.putNumber("Intake Speed", speed);
    }

    // This will return the current speed of the motor
    public double getIntakeSpeed()
    {
        return motorSpeed;
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
