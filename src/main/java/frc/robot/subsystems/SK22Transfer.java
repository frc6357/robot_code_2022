package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ColorSensor;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Constants.TransferConstants;

public class SK22Transfer extends SKSubsystemBase
{
    private CANSparkMax intakeTransferMotor;
    private CANSparkMax exitTransferMotor;
    private CANSparkMax verticalTransferMotor;
    protected boolean verticalFull;

    ColorSensor colorSensor = new ColorSensor(TransferConstants.DISTANCE_THRESHOLD);
    DigitalInput exitTransferSensor;
    DigitalInput verticalTransferSensor;
    // private final I2C.Port i2c = Ports.i2cColorSensor;
    // private ColorSensorV3 colorsensor = new ColorSensorV3(i2c);

    /** Creates a new ExampleSubsystem. */
    public SK22Transfer()
    {
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
        SmartDashboard.putNumber("TransferIntakeMotor", Speed);
    }

    public void setExitTransferMotor(double Speed)
    {
        exitTransferMotor.set(Speed);
        SmartDashboard.putNumber("TransferExitMotor", Speed);
    }

    public void setVerticalTransferMotor(double Speed)
    {
        verticalTransferMotor.set(Speed);
        SmartDashboard.putNumber("TransferVerticalMotor", Speed);
    }

    public void getColorSensor()
    {
        // TODO: Send detected color info back to SmartDashboard.
        colorSensor.getAllColors();
    }

    public boolean getPositionOnePresence()
    {
        boolean State = ((colorSensor
            .getDistance() >= Constants.TransferConstants.DISTANCE_LOW_THRESHOLD)
            && (colorSensor.getDistance() <= Constants.TransferConstants.DISTANCE_HIGH_THRESHOLD));
        SmartDashboard.putBoolean("TransferIntakeBallPresent", State);
        return State;
    }

    public boolean getPositionTwoPresence()
    {
        boolean State = exitTransferSensor.get();
        SmartDashboard.putBoolean("TransferExitBallPresent", State);
        return State;
    }

    public boolean getPositionThreePresence()
    {
        boolean State = verticalTransferSensor.get();
        SmartDashboard.putBoolean("TransferVerticalBallPresent", State);
        return State;
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