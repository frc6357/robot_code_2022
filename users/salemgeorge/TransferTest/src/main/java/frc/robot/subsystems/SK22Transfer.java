package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.ColorSensor;
import frc.robot.Ports;
import frc.robot.Constants.TransferConstants;

public class SK22Transfer extends SKSubsystemBase
{
  public final Alliance teamColor;

  private WPI_VictorSPX intakeTransferMotor;
  private WPI_VictorSPX exitTransferMotor;
  //private WPI_VictorSPX verticalTransferMotor;

  protected boolean verticalFull;
  protected boolean horizontalFull;
  protected boolean isRunningTimer;

  //ColorSensor colorSensor = new ColorSensor(TransferConstants.DISTANCE_THRESHOLD);
  DigitalInput exitTransferSensor;
  DigitalInput verticalTransferSensor;
  private NetworkTable transferEntry;
  // private final I2C.Port i2c = Ports.i2cColorSensor;
  // private ColorSensorV3 colorsensor = new ColorSensorV3(i2c);

  
  /** Creates a new ExampleSubsystem. */
  public SK22Transfer()
  {
      // TODO: This is JUST FOR TESTING
      teamColor = DriverStation.getAlliance();
      
      intakeTransferMotor = new WPI_VictorSPX(Ports.intakeTransferMotor);
      exitTransferMotor = new WPI_VictorSPX(Ports.exitTransferMotor);
      //verticalTransferMotor = new WPI_VictorSPX(Ports.verticalTransferMotor);
      //exitTransferSensor = new DigitalInput(0);
      //verticalTransferSensor = new DigitalInput(0);

  }
  // set all motors, start and stop motors, queue all data
  @Override
  public void periodic()
  {
    
    // This method will be called once per scheduler run
  }

  public void setIntakeTransferMotorSpeed(double Speed)
  {
    intakeTransferMotor.set(Speed);
    System.out.println("Intake: " + Speed);
  }

  public void setExitTransferMotor(double Speed)
  {
    exitTransferMotor.set(-Speed);
    System.out.println("Exit: " + -Speed);
  }

  public void setVerticalTransferMotor(double Speed)
  {
    //verticalTransferMotor.set(Speed);
  }

  public boolean getPositionOnePresence() {
    //return colorSensor.getBallPresence();
    return false;
  }

  public boolean getPositionTwoPresence()
  {
    //return exitTransferSensor.get();
    return false;
  }

  public boolean getPositionThreePresence()
  {
    //return verticalTransferSensor.get();
    return false;
  }

  // Set true if the transfer timer is active
  // The transfer timer keeps track of how many method calls have
  // occurred since the ball has begun changing it's position.
  // For example, the ball is being transfered from intake to the
  // vertical shaft for storage. The transfer timer will be enabled
  // and while it is enabled, the motors required for transfering
  // the ball to that position will be active. When the timer is over,
  // those motors will be disabled.
  public void setTimerState(boolean isEnabled)
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