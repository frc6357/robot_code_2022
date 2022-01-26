package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

// import edu.wpi.first.wpilibj.Encoder;
// import frc.robot.utils.MotorEncoder;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Launcher
{
  private double targetRPM = 0.0;
  //Current rpm of the launcher motors
  private double curRPM = 0f;

  private boolean launcherEnabled = false;

  //Set back to MotorConrollerGroup
  private final MotorController motorControllerGroup;
  // private final MotorEncoder encoder;

  private final double gearRatio;

  //Uncomment
  /** Creates a new SK22Launcher */
  // public Launcher(MotorControllerGroup motorControllerGroup, double gearRatio)
  // {
  //   this.motorControllerGroup = motorControllerGroup;
  //   // this.encoder = encoder;
  //   this.gearRatio = gearRatio;
  // }

  //Delete
  public Launcher(MotorController motorController, double gearRatio) {
    this.motorControllerGroup = motorController;
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

    motorControllerGroup.set(convertRPMtoMotorInput(targetRPM));
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
    // TODO: just for testing
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

    // TODO: Nasty hack, just to test prototype(will fix)
    if(isLauncherEnabled()) 
    {
      motorControllerGroup.set(convertRPMtoMotorInput(targetRPM));
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

  private double convertRPMtoMotorInput(double RPM) {
    // TODO: just for testing
    return (RPM / 10000);
  }
}