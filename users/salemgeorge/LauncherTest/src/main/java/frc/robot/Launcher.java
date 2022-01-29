package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

// import edu.wpi.first.wpilibj.Encoder;
// import frc.robot.utils.MotorEncoder;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Launcher
{
  //TODO: Add method to calculate RPM

  private double targetRPM = 0.0;
  //Current rpm of the launcher motors

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
    // Get the current rotation rate in pulses per 100mS
    
    double launcherRPM = getLauncherRPM();

    System.out.println("LauncherRPM: " + launcherRPM);

    // motorControllerGroup.set(bangBangController.calculate(launcherRPM, targetRPM));
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

  public double getLauncherRPM() 
  {
    double pulsesPerMinute = motorEncoder1.getVelocityPulses() * 600;
    double MotorRevsPerMinute = pulsesPerMinute / Constants.DriveConstants.ENCODER_CPR;
    double LauncherRPM = MotorRevsPerMinute / Constants.LauncherConstants.LAUNCH_GEAR_RATIO;

    return LauncherRPM;
  }

  private double convertRPMtoMotorInput(double RPM) {
    // TODO: just for testing
    return (RPM / 10000);
  }
}