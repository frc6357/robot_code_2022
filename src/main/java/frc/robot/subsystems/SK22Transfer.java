package frc.robot.subsystems;

import javax.print.CancelablePrintJob;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.ColorSensor;
import frc.robot.Constants;
import frc.robot.Constants.TransferConstants;
import frc.robot.Ports;
import frc.robot.commands.DefaultTransferCommand;
import frc.robot.subsystems.base.SwitchSensor;

/**
 * Class controlling the ball transfer subsystem in the 2022 robot.
 */
public class SK22Transfer extends SKSubsystemBase
{
    // TODO: Don't use public members. Move this to the class that actually needs it.
    public final Alliance teamAlliance;

    private CANSparkMax intakeTransferMotor;
    private CANSparkMax exitTransferMotor;
    // This is the motor that accepts the ball from the horizontal shaft
    private CANSparkMax verticalTransferMotor;

    protected boolean isRunningTimer = false;

    ColorSensor  colorSensor = new ColorSensor(TransferConstants.DISTANCE_THRESHOLD);
    SwitchSensor exitTransferSensor;
    SwitchSensor verticalTransferSensor;

    // private final I2C.Port i2c = Ports.i2cColorSensor;
    // private ColorSensorV3 colorsensor = new ColorSensorV3(i2c);

    /**
     * Constructor for the ball transfer subsystem class.
     */
    public SK22Transfer()
    {
        // TODO: This is not used by this class. Move it to the class that actually
        // needs it (DefaultTransferCommand).
        teamAlliance = DriverStation.getAlliance();

        intakeTransferMotor = new CANSparkMax(Ports.INTAKE_TRANSFER_MOTOR, MotorType.kBrushless);
        exitTransferMotor = new CANSparkMax(Ports.EXIT_TRANSFER_MOTOR, MotorType.kBrushless);
        verticalTransferMotor =
                new CANSparkMax(Ports.VERTICAL_TRANSFER_MOTOR, MotorType.kBrushless);

        exitTransferSensor = new SwitchSensor(Ports.EXIT_SENSOR,
            Constants.TransferConstants.EXIT_SENSOR_POLARITY);
        verticalTransferSensor = new SwitchSensor(Ports.VERTICAL_SENSOR,
            Constants.TransferConstants.VERTICAL_SENSOR_POLARITY);

        verticalTransferMotor.setIdleMode(IdleMode.kBrake);

        intakeTransferMotor.setInverted(true);

        setDefaultCommand(new DefaultTransferCommand(this));
    }

    // set all motors, start and stop motors, queue all data
    @Override
    public void periodic()
    {
        colorSensor.periodic();
    }

    /**
     * Set the speed of the intake-side transfer motor.
     * 
     * @param speed
     *            Motor speed in the range [-1, 1].
     */
    public void setIntakeTransferMotor(double speed)
    {
        intakeTransferMotor.set(speed);
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

    public CANSparkMax getIntakeTransferMotor()
    {
        return intakeTransferMotor;
    }

    public CANSparkMax getExitTransferMotor()
    {
        return exitTransferMotor;
    }

    public CANSparkMax getVerticalShaftMotor()
    {
        return verticalTransferMotor;
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

    // TODO: This should be owned by the launcher class now assuming its operation is
    // independent of the vertical transfer motor and is only needed to control moving
    // a ball into the launcher proper.

    /**
     * Query the current color detected by the color sensor.
     * 
     * @return "Red", "Blue" or null if no ball is detected.
     */
    public String getColorSensor()
    {
        return colorSensor.getColor();
    }

    /**
     * States if there is a ball in the intake part of the transfer
     * 
     * @return Whether there is a ball in position one.
     */
    public boolean getPositionOnePresence()
    {
        return colorSensor.getBallPresence();
    }

    // TODO: Rename these methods to make it clear which positions "PositionTwo"
    // and "PositionThree" refer to. 

    /**
     * States if there is a ball in the eject part of the transfer
     * 
     * @return Whether there is a ball in position two.
     */
    public boolean getPositionTwoPresence()
    {
        return exitTransferSensor.get();
    }

    /**
     * States if there is a ball in the launcher part of the transfer
     * 
     * @return Whether there is a ball in position three.
     */
    public boolean getPositionThreePresence()
    {
        return verticalTransferSensor.get();
    }

    // TODO: The name of this method is weird. "set" implies a method
    // that changes something but "IsRunningTimerEnabled" sounds like
    // a query. What does this do? Also, I don't see anything in the
    // periodic method that updates a timer counter.

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
