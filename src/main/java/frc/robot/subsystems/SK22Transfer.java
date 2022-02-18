package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.ColorSensor;
import frc.robot.Constants;
import frc.robot.Constants.TransferConstants;
import frc.robot.commands.DefaultTransferCommand;
import frc.robot.subsystems.base.SwitchSensor;
import frc.robot.Ports;

public class SK22Transfer extends SKSubsystemBase
{
  public final Alliance teamAlliance;

  private VictorSPX intakeTransferMotor;
  private VictorSPX exitTransferMotor;
  // This is the motor that accepts the ball from the horizontal shaft
  private CANSparkMax verticalTransferMotor;
  // This is the motor that transfers the ball from the vertical shaft
  // into the launcher
  private CANSparkMax launcherTransferMotor;

  protected boolean isRunningTimer;

  ColorSensor colorSensor = new ColorSensor(TransferConstants.DISTANCE_THRESHOLD);
  SwitchSensor exitTransferSensor;
  SwitchSensor verticalTransferSensor;
  private NetworkTable transferEntry;
  // private final I2C.Port i2c = Ports.i2cColorSensor;
  // private ColorSensorV3 colorsensor = new ColorSensorV3(i2c);

  /** Creates a new ExampleSubsystem. */
  public SK22Transfer()
  {
      // TODO: This is JUST FOR TESTING
      teamAlliance = DriverStation.getAlliance();

      intakeTransferMotor = new VictorSPX(Ports.INTAKE_TRANSFER_MOTOR);
      exitTransferMotor = new VictorSPX(Ports.EXIT_TRANSFER_MOTOR);
    
      verticalTransferMotor = new CANSparkMax(Ports.VERTICAL_TRANSFER_MOTOR, MotorType.kBrushless);
      launcherTransferMotor = new CANSparkMax(Ports.LAUNCHER_TRANSFER_MOTOR, MotorType.kBrushless);

      exitTransferSensor = new SwitchSensor(Ports.EXIT_SENSOR, Constants.TransferConstants.EXIT_SENSONR_POLARITY);
      verticalTransferSensor = new SwitchSensor(Ports.VERTICAL_SENSOR, Constants.TransferConstants.VERTICAL_SENSOR_POLARITY);

      verticalTransferMotor.setIdleMode(IdleMode.kCoast);
      launcherTransferMotor.setIdleMode(IdleMode.kCoast);

      setDefaultCommand(new DefaultTransferCommand(this));
  }

  // set all motors, start and stop motors, queue all data
  @Override
  public void periodic()
  {
    colorSensor.periodic();
    // This method will be called once per scheduler run
  }

  public void setIntakeTransferMotor(double speed)
  {
    intakeTransferMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void setExitTransferMotor(double speed)
  {
    exitTransferMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void setVerticalTransferMotor(double speed)
  {
    verticalTransferMotor.set(speed);
  }

  public boolean getVerticalTransferMotorEnabled()
  {
    return verticalTransferMotor.get() != 0;
  }

  public void setLauncherTransferMotor(double speed)
  {
    launcherTransferMotor.set(speed);
  }

  public String getColorSensor()
  {
    return colorSensor.getColor();
  }

  /**
   * States if there is a ball in the intake part of the transfer
   * @return Whether there is a ball in position one.
   */
  public boolean getPositionOnePresence() {
    return colorSensor.getBallPresence();
  }

  /**
   * States if there is a ball in the eject part of the transfer
   * @return Whether there is a ball in position two.
   */
  public boolean getPositionTwoPresence()
  {
    return exitTransferSensor.get();
  }

  /**
   * States if there is a ball in the launcher part of the transfer
   * @return Whether there is a ball in position three.
   */
  public boolean getPositionThreePresence()
  {
    return verticalTransferSensor.get();
  }

  // Set true if the transfer timer is active
  // The transfer timer keeps track of how many method calls have
  // occurred since the ball has begun changing it's position.
  // For example, the ball is being transfered from intake to the
  // vertical shaft for storage. The transfer timer will be enabled
  // and while it is enabled, the motors required for transfering
  // the ball to that position will be active. When the timer is over,
  // those motors will be disabled.
  public void setIsRunningTimerEnabled(boolean isEnabled)
  {
    isRunningTimer = isEnabled;
  }

  public boolean getIsRunningTimerEnabled() 
  {
    return isRunningTimer;
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