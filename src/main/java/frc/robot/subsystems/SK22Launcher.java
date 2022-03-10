package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.base.Launcher;
import frc.robot.subsystems.base.MotorEncoder;

/**
 * Class controlling the launcher subsystem in the 2022 robot.
 */
public class SK22Launcher extends SubsystemBase
{
  private final WPI_TalonFX ballLauncher1 = new WPI_TalonFX(Ports.BALL_LAUNCHER_1);
  private final WPI_TalonFX ballLauncher2 = new WPI_TalonFX(Ports.BALL_LAUNCHER_2);

  // This is the motor at the base of the vertical transfer shaft.
  private CANSparkMax exitTransferMotor;

  // This is the motor that moves the ball up the vertical transfer shaft
  private CANSparkMax verticalTransferMotor;

  // This is the motor that transfers the ball from the vertical shaft
  // into the launcher
  private CANSparkMax launcherTransferMotor;

  private final MotorControllerGroup launcherGroup =
      new MotorControllerGroup(ballLauncher1, ballLauncher2);

  private final MotorEncoder motorEncoder1 =
      new MotorEncoder(ballLauncher1, Constants.LauncherConstants.ENCODER_DISTANCE_PER_PULSE,
        Constants.LauncherConstants.LEFT_ENCODER_REVERSED);

  private final Launcher launcher;

  private double setpoint = 0.0;
  private double speed = 0.0;

  /** Creates a new ExampleSubsystem. */
  public SK22Launcher()
  {
    exitTransferMotor     = new CANSparkMax(Ports.EXIT_TRANSFER_MOTOR,     MotorType.kBrushless);
    verticalTransferMotor = new CANSparkMax(Ports.VERTICAL_TRANSFER_MOTOR, MotorType.kBrushless);
    launcherTransferMotor = new CANSparkMax(Ports.LAUNCHER_TRANSFER_MOTOR, MotorType.kBrushless);

    ballLauncher1.setNeutralMode(NeutralMode.Coast);
    ballLauncher1.setInverted(true);
    ballLauncher1.configOpenloopRamp(2);
    ballLauncher2.setNeutralMode(NeutralMode.Coast);
    ballLauncher2.setInverted(false);
    ballLauncher2.configOpenloopRamp(2);

    verticalTransferMotor.setIdleMode(IdleMode.kBrake);

    SmartDashboard.putNumber("Launcher Setpoint RPM", 0.0);
    SmartDashboard.putBoolean("Launcher at Setpoint RPM", false);

    launcherTransferMotor.setIdleMode(IdleMode.kBrake);
    launcherTransferMotor.setInverted(true);

    launcher =
        new Launcher(launcherGroup, motorEncoder1, Constants.LauncherConstants.LAUNCH_GEAR_RATIO,
          Constants.LauncherConstants.LAUNCHER_ENCODER_CPR, Constants.LauncherConstants.LAUNCHER_KP,
          Constants.LauncherConstants.LAUNCHER_KI, Constants.LauncherConstants.LAUNCHER_KD);

    launcher.setTargetMotorRPM(0.0);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    launcher.update();
    setpoint = launcher.getTargetMotorRPM();
    speed = launcher.getLauncherRPM();

    SmartDashboard.putNumber("Launcher RPM", speed);
    SmartDashboard.putBoolean("Launcher at Setpoint RPM", Math.abs(setpoint - speed) < 100.0);
  }

  @Override
  public void simulationPeriodic()
  {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Start the launcher flywheel motor.
   */
  public void enableLauncher()
  {
    launcher.enableLauncher();
  }

  /**
   * Stop the launcher flywheel motor.
   */
  public void disableLauncher()
  {
    launcher.disableLauncher();
  }

  /**
   * Set the desired launcher flywheel speed.
   * 
   * @param rpm
   *          The speed that the launcher will be set to run at
   */
  public void setLauncherRPM(double rpm)
  {
    launcher.setTargetMotorRPM(rpm);
  }

  /**
   * Gets the last set speed 
   * @return The value of the last set speed in RPM
   */
  public double getLauncherRPMSetpoint()
  {
    return setpoint;
  }

  /**
   * Gets the current speed of the launcher
   * @return The value of the speed in RPM
   */
  public double getLauncherRPM()
  {
    return speed;
  }

    /**
   * Set the speed of the exit-side transfer motor.
   * 
   * @param speed
   *            Motor speed in the range [-1, 1].
   */
  public void setExitTransferMotor(double speed)
  {
      exitTransferMotor.set(speed);
  }

    /**
   * Set the speed of the vertical transfer motor.
   * 
   * @param speed
   *            Motor speed in the range [-1, 1].
   */
  public void setVerticalTransferMotor(double speed)
  {
      verticalTransferMotor.set(speed);
  }

  /**
   * Set the speed of the launcher transfer motor.
   * 
   * @param speed
   *          Motor speed in the range [-1, 1].
   */
  public void setLauncherTransferMotor(double speed)
  {
    launcherTransferMotor.set(speed);
  }
  
  /**
   * Query whether the vertical transfer motor is running.
   * 
   * @return true if the motor has non-zero speed, false otherwise.
   */
  public boolean getVerticalTransferMotorEnabled()
  {
      return verticalTransferMotor.get() != 0;
  }

  
}
