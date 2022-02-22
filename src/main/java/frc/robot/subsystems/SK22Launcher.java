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

  private final MotorControllerGroup launcherGroup =
      new MotorControllerGroup(ballLauncher1, ballLauncher2);

  private final MotorEncoder motorEncoder1 =
      new MotorEncoder(ballLauncher1, Constants.LauncherConstants.ENCODER_DISTANCE_PER_PULSE,
        Constants.LauncherConstants.LEFT_ENCODER_REVERSED);

  // This is the motor that transfers the ball from the vertical shaft
  // into the launcher
  private CANSparkMax launcherTransferMotor =
      new CANSparkMax(Ports.LAUNCHER_TRANSFER_MOTOR, MotorType.kBrushless);

  private final Launcher launcher;

  /** Creates a new ExampleSubsystem. */
  public SK22Launcher()
  {
    ballLauncher1.setNeutralMode(NeutralMode.Coast);
    ballLauncher1.setInverted(true);
    ballLauncher2.setNeutralMode(NeutralMode.Coast);
    ballLauncher2.setInverted(false);

    launcherTransferMotor.setIdleMode(IdleMode.kBrake);

    launcher =
        new Launcher(launcherGroup, motorEncoder1, Constants.LauncherConstants.LAUNCH_GEAR_RATIO,
          Constants.LauncherConstants.LAUNCHER_ENCODER_CPR, Constants.LauncherConstants.LAUNCHER_KP,
          Constants.LauncherConstants.LAUNCHER_KI, Constants.LauncherConstants.LAUNCHER_KD);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    launcher.update();
    double setpoint = launcher.getTargetMotorRPM();
    double speed = launcher.getCurMotorRPM();

    SmartDashboard.putNumber("Launcher RPM", speed);
    SmartDashboard.putNumber("Launcher Setpoint RPM", setpoint);
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
   * Set the speed of the launcher transfer motor.
   * 
   * @param speed
   *          Motor speed in the range [-1, 1].
   */
  public void setLauncherTransferMotor(double speed)
  {
    launcherTransferMotor.set(speed);
  }

}
