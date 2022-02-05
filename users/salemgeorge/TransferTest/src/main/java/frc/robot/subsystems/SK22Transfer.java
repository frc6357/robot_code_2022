package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.ColorSensor;
import frc.robot.Ports;
import frc.robot.Constants.TransferConstants;

public class SK22Transfer extends SKSubsystemBase
{
  public final String teamColor;

  private CANSparkMax intakeTransferMotor;
  private CANSparkMax exitTransferMotor;
  private CANSparkMax verticalTransferMotor;

  protected boolean verticalFull;
  protected boolean horizontalFull;
  protected boolean isRunningTimer;

  ColorSensor colorSensor = new ColorSensor(TransferConstants.DISTANCE_THRESHOLD);
  DigitalInput exitTransferSensor;
  DigitalInput verticalTransferSensor;
  private NetworkTable transferEntry;
  // private final I2C.Port i2c = Ports.i2cColorSensor;
  // private ColorSensorV3 colorsensor = new ColorSensorV3(i2c);

  
  /** Creates a new ExampleSubsystem. */
  public SK22Transfer()
  {
      // TODO: This is JUST FOR TESTING
      teamColor = "Red";

      intakeTransferMotor = new CANSparkMax(Ports.intakeTransferMotor, MotorType.kBrushless);
      exitTransferMotor = new CANSparkMax(Ports.exitTransferMotor, MotorType.kBrushless);
      verticalTransferMotor = new CANSparkMax(Ports.verticalTransferMotor, MotorType.kBrushless);
      exitTransferSensor = new DigitalInput(0);
      verticalTransferSensor = new DigitalInput(0);

  }
  // set all motors, start and stop motors, queue all data
  @Override
  public void periodic()
  {
    
    // This method will be called once per scheduler run
  }

  public void setIntakeTransferMotor(double Speed)
  {
    intakeTransferMotor.set(Speed);
  }

  public void setExitTransferMotor(double Speed)
  {
    exitTransferMotor.set(Speed);
  }

  public void setVerticalTransferMotor(double Speed)
  {
    verticalTransferMotor.set(Speed);
  }

  public String getColorSensor()
  {
    return colorSensor.getColor();
  }

  public boolean getPositionOnePresence() {
    return colorSensor.getBallPresence();
  }

  public boolean getPositionTwoPresence()
  {
    return exitTransferSensor.get();
  }

  public boolean getPositionThreePresence()
  {
    return verticalTransferSensor.get();
  }

  public void setVerticalFull(boolean isFull)
  {
    verticalFull = isFull;
  }

  public void setHorizontalFull(boolean isFull)
  {
    horizontalFull = isFull;
  }

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