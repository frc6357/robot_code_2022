//
//SK22Intake: This is the system for getting cargo into the robot in the 2022 season.
//

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Ports;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class SK22Intake extends SKSubsystemBase
{
  private final DoubleSolenoid intakeMover = new DoubleSolenoid(Ports.intakePCM, Ports.intakeMoverForward, Ports.intakeMoverBackward);
  private final CANSparkMax intakeMotor = new CANSparkMax(Ports.intakeMotor, MotorType.kBrushless);
  
  /** Creates a new SK22Intake. */
  public SK22Intake()
  {
    disableIntake();
  }
  /** This should be the safe and initial code that happens. Nothing should move. */
  public void disableIntake()
  {
    intakeMover.set(Value.kOff);
    intakeMotor.set(0.0);
  }
  /** This will extend the double solenoids, moving the intake to the active position,
   *  then, will turn on the intake motors. */
  public void extendIntake()
  {
    intakeMover.set(Value.kForward);
  }
  /** This will retract the double solenoids, moving the intake to the inactive position then, 
   * turning off the intake motor shortly after completion.*/
  public void retractIntake()
  {
    intakeMover.set(Value.kReverse);
  }
  /** this will set the speed of the motor that powers the ball pulling part of the intake
   */
  public void setIntakeSpeed(double speed)
  {
    intakeMotor.set(speed);
  }
  /** This will display the current speed of the motor */
  public double getIntakeSpeed()
  {
    return(intakeMotor.get());
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
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