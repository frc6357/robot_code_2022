package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

public class Launcher
{
  private double targetRPM        = 0.0;
  private boolean launcherEnabled = false;
  private double EncoderCPR       = 0.0;

  private final MotorControllerGroup motorControllerGroup;
  private final MotorEncoder motorEncoder1;
  // private final MotorEncoder motorEncoder2;

  private double motorOutput;

  private final PIDController Controller;

  private final double gearRatio;

  // TODO: When second motor is added to the launcher, make sure to add a motorEncoder2 to this constructor
  /** Creates a new SK22Launcher */
  public Launcher(MotorControllerGroup motorControllerGroup, MotorEncoder motorEncoder1, double gearRatio, int encoderCPR, double KP, double KI, double KD)
  {
    this.motorControllerGroup = motorControllerGroup;
    this.motorEncoder1 = motorEncoder1;
    this.gearRatio = gearRatio;
    this.EncoderCPR   = encoderCPR;

    this.Controller = new PIDController(KP, KI, KD);
  }
  
  public void update()
  {
    double launcherRPM = getLauncherRPM();

    // System.out.println("Target: " + targetRPM + " Actual: " + launcherRPM);

    SmartDashboard.putNumber("Launcher RPM", launcherRPM);
    

    if(launcherEnabled || (targetRPM == 0.0))
    {
      
      motorOutput = Controller.calculate(launcherRPM, targetRPM);
      if(motorOutput < 0.0){
        motorOutput = 0.0;
      }
      motorControllerGroup.set(motorOutput);
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

  public void resetController(double kP, double kI, double kD){
    Controller.setPID(kP, kI, kD);
    
  }

  public void reset(){
    Controller.reset();
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
    double MotorRevsPerMinute = pulsesPerMinute / EncoderCPR;
    double LauncherRPM = MotorRevsPerMinute / gearRatio;

    return LauncherRPM;
  }
}