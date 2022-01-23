package frc.robot;

// import edu.wpi.first.wpilibj.Encoder;
import frc.robot.utils.MotorEncoder;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Launcher
{
  private double targetRPM = 0.0;
  //Current rpm of the launcher motors
  private double curRPM = 0f;

  private boolean launcherEnabled = false;

  private final MotorControllerGroup motorControllerGroup;
  private final MotorEncoder encoder;

  private final double gearRatio;

  /** Creates a new SK22Launcher */
  public Launcher(MotorControllerGroup motorControllerGroup, MotorEncoder encoder, double gearRatio)
  {
    this.motorControllerGroup = motorControllerGroup;
    this.encoder = encoder;
    this.gearRatio = gearRatio;
  }
  
  public void update() 
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

    motorControllerGroup.set(targetRPM);
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

    motorControllerGroup.set(0.0);
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

    // TODO: Nasty hack, just to test prototype(will fix)
    if(isLauncherEnabled()) 
    {
      motorControllerGroup.set(targetRPM);
    }

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