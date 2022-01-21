package frc.robot.subsystems;



public class SK22Launcher extends SKSubsystemBase
{
  private double targetRPM = 4000f;
  //Current rpm of the launcher motors
  private double curRPM = 0f;

  private boolean launcherEnabled = false;

  /** Creates a new SK22Launcher */
  public SK22Launcher()
  {
    
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

  
  /**
   * 
   * Puts the launcher into the enabled state
   * The launcher motors will be powered on to reach target rpm
   * 
   */
  public void enableLauncher() 
  {
    launcherEnabled = true;
  }

  /**
   * 
   * Puts the launcher into a disabled state
   * The launcher motors will coast
   * 
   */
  public void disableLauncher() 
  {
    launcherEnabled = false;
  }

  /**
   * 
   * Returns true if the launcher is enabled
   * Returns false if the launcher is disabled
   * 
   * @return Is launcher enabled?
   * 
   */
  public boolean isLauncherEnabled() 
  {
    return launcherEnabled;
  }

  /**
   * 
   * Returns the current rpm of the launcher motors
   * 
   * @return Current rpm of the launcher motors
   * 
   */
  public double getCurMotorRPM() 
  {
    return curRPM;
  }

  /**
   * 
   * Sets the target rpm of the launcher motors
   * 
   * @param targetRPM
   *                  When the launcher is in the enabled state, the launcher motors will attempt to reach this speed
   */
  public void setTargetMotorRPM(double targetRPM) 
  {
    this.targetRPM = targetRPM;
  }

  /**
   * 
   * @return Gets the target rpm
   */
  public double getTargetMotorRPM() 
  {
    return targetRPM;
  }
}