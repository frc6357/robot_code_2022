package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.ColorSensor;
import frc.robot.Ports;

public class SK22Transfer extends SKSubsystemBase
{

  private CANSparkMax horizontalTransferMotor1 = new CANSparkMax(Ports.horizontalTransferMotor1, MotorType.kBrushless);
  private CANSparkMax horizontalTransferMotor2 = new CANSparkMax(Ports.horizontalTransferMotor2,MotorType.kBrushless);
  private CANSparkMax VerticalTransferMotor = new CANSparkMax(Ports.VerticalTransferMotor ,MotorType.kBrushless);
  ColorSensor cs1 = new ColorSensor(70);
  // private final I2C.Port i2c = Ports.i2cColorSensor;
  // private ColorSensorV3 colorsensor = new ColorSensorV3(i2c);

  
  /** Creates a new ExampleSubsystem. */
  public SK22Transfer()
  {
      
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
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