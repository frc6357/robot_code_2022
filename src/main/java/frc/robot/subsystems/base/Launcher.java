package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

/**
 * A generic flywheel-based ball launcher
 */
public class Launcher
{
  private double targetRPM        = 0.0;
  private boolean launcherEnabled = false;
  private double encoderCPR       = 0.0;

  private final MotorControllerGroup motorControllerGroup;
  private final MotorEncoder motorEncoder1;

  private final PIDController controller;

  private final double gearRatio;

  /** Creates a new Launcher
   * 
   * Note that we only pass one motor encoder here. Both motors are operated as a group. 
   * We don't control them independently but run one PID controller with input based on
   * the reading from one encoder.
   * 
   * @param motorControllerGroup The motor controller group driving the flywheel
   * @param motorEncoder1 The encoder used to measure flywheel speed
   * @param gearRatio The gear reduction ration from motor to flywheel
   * @param encoderCPR The encoder counts per revolution
   * @param kP PID controlller proportional gain
   * @param kI PID controller integral gain
   * @param kD PID controller differential gain
   */
  public Launcher(MotorControllerGroup motorControllerGroup,
                  MotorEncoder motorEncoder1,
                  double gearRatio,
                  int encoderCPR,
                  double kP,
                  double kI,
                  double kD)
  {
    this.motorControllerGroup = motorControllerGroup;
    this.motorEncoder1 = motorEncoder1;
    this.gearRatio = gearRatio;
    this.encoderCPR   = encoderCPR;

    this.controller = new PIDController(kP, kI, kD);
  }
  
  /**
   * Periodic update function for the launcher. This must be called periodically
   * (every 20mS or so) to run the launcher control loop.
   */
  public void update()
  {
    double launcherRPM = getLauncherRPM();

    // System.out.println("Target: " + targetRPM + " Actual: " + launcherRPM);

    SmartDashboard.putNumber("Launcher RPM", launcherRPM);
    
    if (launcherEnabled || (targetRPM == 0.0))
    {
      
      double motorOutput = controller.calculate(launcherRPM, targetRPM);
      if (motorOutput < 0.0)
      {
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

  public void resetController(double kP, double kI, double kD)
  {
    controller.setPID(kP, kI, kD);
  }

  public void reset()
  {
    controller.reset();
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
    if (isLauncherEnabled()) 
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

  /**
   * Query the current speed of the launcher flywheel.
   * @return The current launcher speed in revs per minute.
   */
  public double getLauncherRPM() 
  {
    double pulsesPerMinute = motorEncoder1.getVelocityPulses() * 600;
    double motorRevsPerMinute = pulsesPerMinute / encoderCPR;
    double launcherRPM = motorRevsPerMinute / gearRatio;

    return launcherRPM;
  }
}
