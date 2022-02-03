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

  private CANSparkMax horizontalTransferMotor1;
  private CANSparkMax horizontalTransferMotor2;
  private CANSparkMax verticalTransferMotor;
  protected boolean verticalFull;

  ColorSensor colorSensor = new ColorSensor(TransferConstants.DISTANCE_THRESHOLD);
  DigitalInput horizontalSensor;
  DigitalInput verticalSensor = new DigitalInput(0);
  private NetworkTable transferEntry;
  // private final I2C.Port i2c = Ports.i2cColorSensor;
  // private ColorSensorV3 colorsensor = new ColorSensorV3(i2c);

  
  /** Creates a new ExampleSubsystem. */
  public SK22Transfer()
  {
      horizontalTransferMotor1 = new CANSparkMax(Ports.horizontalTransferMotor1, MotorType.kBrushless);
      horizontalTransferMotor2 = new CANSparkMax(Ports.horizontalTransferMotor2, MotorType.kBrushless);
      verticalTransferMotor = new CANSparkMax(Ports.verticalTransferMotor, MotorType.kBrushless);
      horizontalSensor = new DigitalInput(0);
      verticalSensor = new DigitalInput(0);

  }
  // set all motors, start and stop motors, queue all data
  @Override
  public void periodic()
  {
    
    // This method will be called once per scheduler run
  }

  public void setPositionOneMotor(double Speed)
  {
    horizontalTransferMotor1.set(Speed);
  }

  public void setPositionTwoMotor(double Speed)
  {
    horizontalTransferMotor2.set(Speed);
  }

  public void setPositionThreeMotor(double Speed)
  {
    verticalTransferMotor.set(Speed);
  }

  public void stopPositionOneMotor()
  {
    horizontalTransferMotor1.set(TransferConstants.STOP_SPEED);
  }

  public void stopPositionTwoMotor()
  {
    horizontalTransferMotor2.set(TransferConstants.STOP_SPEED);
  }

  public void stopPositionThreeMotor()
  {
    verticalTransferMotor.set(TransferConstants.STOP_SPEED);
  }


  public void getColorSensor()
  {
    colorSensor.getAllColors();
  }

  public boolean getPositionTwoPrecense()
  {
    return horizontalSensor.get();
  }

  public boolean getPositionThreePrecense()
  {
    return verticalSensor.get();
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