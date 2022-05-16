package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Ports;
import frc.robot.commands.DefaultTransferCommand;
import frc.robot.subsystems.base.SwitchSensor;

/**
 * Class controlling the ball transfer subsystem in the 2022 robot.
 */
public class SK22Transfer extends SKSubsystemBase
{
    private CANSparkMax intakeTransferMotor;

    protected boolean isRunningTimer = false;

    // ColorSensor  colorSensor = new ColorSensor(TransferConstants.DISTANCE_THRESHOLD);
    SwitchSensor exitTransferSensor;
    SwitchSensor verticalTransferSensor;

    // private final I2C.Port i2c = Ports.i2cColorSensor;
    // private ColorSensorV3 colorsensor = new ColorSensorV3(i2c);

    /**
     * Constructor for the ball transfer subsystem class.
     */
    public SK22Transfer()
    {
        intakeTransferMotor = new CANSparkMax(Ports.INTAKE_TRANSFER_MOTOR, MotorType.kBrushless);
        intakeTransferMotor.setInverted(true);
        setDefaultCommand(new DefaultTransferCommand(this));
    }

    // set all motors, start and stop motors, queue all data
    @Override
    public void periodic()
    {
        // colorSensor.periodic();
    }

    /**
     * Set the speed of the intake-side transfer motor.
     * 
     * @param speed
     *            Motor speed in the range [-1, 1].
     */
    public void setIntakeTransferMotorSpeed(double speed)
    {
        intakeTransferMotor.set(speed);
    }



    /**
     * returns the value of the transfer intake motor
     * @return Value of the transfer intake motor
     */
    public CANSparkMax getIntakeTransferMotor()
    {
        return intakeTransferMotor;
    }

    /**
     * Query the current color detected by the color sensor.
     * 
     * @return "Red", "Blue" or null if no ball is detected.
     */
    public String getColorSensor()
    {
        return null;
        // return colorSensor.getColor();
    }

    /**
     * Set true if the transfer timer is active
     * 
     * The transfer timer keeps track of how many periodic method calls have occurred
     * since the ball has begun changing its position. For example, the ball is being
     * transfered from intake to the vertical shaft for storage. The transfer timer will
     * be enabled and while it is enabled, the motors required for transfering the ball to
     * that position will be active. When the timer is over, those motors will be
     * disabled.
     * 
     * @param isEnabled
     *            Unclear what this is for
     */
    public void setTimerState(boolean isEnabled)
    {
        isRunningTimer = isEnabled;
    }

    /**
     * 
     * @return True of the timer is enabled, False if disabled.
     */
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
