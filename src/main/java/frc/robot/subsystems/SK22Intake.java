//
//SK22Intake: This is the system for getting cargo into the robot in the 2022 season.
//

package frc.robot.subsystems;


public class SK22Intake extends SKSubsystemBase
{
  /** Creates a new SK22Intake. */
  public SK22Intake()
  {

  }
  //this should be the safe and initial code that happens. nothing should move
  public void disableIntake()
  {
    
  }
  //this will extend the double solenoids, moving the intake to the active position
  public void extendIntake()
  {

  }
  //his will retract the double solenoids, moving the intake to the inactive position
  public void retractIntake()
  {

  }
  //this will set the speed of the motor that powers the ball pulling part of the intake
  public void setIntakeSpeed(double speed) 
  {

  }
  //this will display the current speed of the motor
  public double getIntakeSpeed()
  {
    return 0.0;
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