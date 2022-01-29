package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.BangBangController;

public class Launcher
{
  private double targetRPM = 0.0;
  private boolean launcherEnabled = false;

  private final MotorControllerGroup motorControllerGroup;
  private final MotorEncoder motorEncoder1;
  // private final MotorEncoder motorEncoder2;

  private final BangBangController bangBangController;

  private final double gearRatio;

  // TODO: When second motor is added to the launcher, make sure to add a motorEncoder2 to this constructor
  /** Creates a new SK22Launcher */
  public Launcher(MotorControllerGroup motorControllerGroup, MotorEncoder motorEncoder1, double gearRatio)
  {
    this.motorControllerGroup = motorControllerGroup;
    this.motorEncoder1 = motorEncoder1;
    this.gearRatio = gearRatio;

    this.bangBangController = new BangBangController();
  }
  
  public void update()
  {
    double launcherRPM = getLauncherRPM();

    System.out.println("LauncherRPM: " + launcherRPM);

    if(launcherEnabled)
    {
      motorControllerGroup.set(bangBangController.calculate(launcherRPM, targetRPM));
    }
    else
    {
      motorControllerGroup.set(0.0);
    }
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
    if(isLauncherEnabled()) 
    {
      return targetRPM;
    } 
    else 
    {
      return 0.0;
    }
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

  public double getLauncherRPM() 
  {
    double pulsesPerMinute = motorEncoder1.getVelocityPulses() * 600;
    // TODO: Update constructor to pass in encoder CPR number.
    double MotorRevsPerMinute = pulsesPerMinute / Constants.DriveConstants.ENCODER_CPR;
    double LauncherRPM = MotorRevsPerMinute / gearRatio;

    return LauncherRPM;
  }
}