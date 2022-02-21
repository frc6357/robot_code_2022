package frc.robot.subsystems.base;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Generic PID controlled rotating arm
 */
public class RotatingArm
{

    private final PIDController   controller;
    private final CANSparkMax     motor;
    private final RelativeEncoder encoder;
    private final double          motorRotationsToDegrees;
    private double                setPoint;
    private boolean               armEnabled = false;

    /**
     * Constructor method for rotating arm class Positions of the arm is considred
     * relative to the arm's position.
     * 
     * @param motor
     *            CANSparkMax motor controller that controls the pivot of the arm
     * @param encoder
     *            RelativeEncoder that measures arm angle
     * @param motorRotationsToDegrees
     *            conversion factor between the motor rotations and the arm angle in
     *            degrees Degrees equals motors postions times
     * @param kP
     *            PID controller porportional gain
     * @param kI
     *            PID controller integral gain
     * @param kD
     *            PID controller derivative gain
     */
    public RotatingArm(CANSparkMax motor, RelativeEncoder encoder, double motorRotationsToDegrees,
        double kP, double kI, double kD)
    {
        setPoint = 0.0;
        this.motor = motor;
        this.encoder = encoder;
        this.motorRotationsToDegrees = motorRotationsToDegrees;
        this.controller = new PIDController(kP, kI, kD);

        this.encoder.setPosition(0.0);
        this.encoder.setPositionConversionFactor(this.motorRotationsToDegrees);
        this.motor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Move to certain position relative to start location
     * 
     * @param degrees
     *            desired position of arm in degrees
     */
    public void goToAngle(double degrees)
    {
        this.setPoint = degrees;
    }

    /**
     * Return the value of the set point.
     * @return current arm setpoint in degrees relative to the start position.
     */
    public double getSetPointAngle()
    {
        return setPoint;
    }

    /**
     * Find the current angle.
     * 
     * @return angle of arm
     */
    public double getAngle()
    {
        // Expects rotations instead of degrees. Must do the converstion outside of method. Get poistion returns motor
        // rotations however we have scaled this to fit for rotations instead.
        return encoder.getPosition();
    }

    /**
     * Starts control of the arm
     */
    public void enable()
    {
        armEnabled = true;
    }

    /**
     * Stops contorl of the arm
     */
    public void disable()
    {
        armEnabled = false;
    }

    /**
     * Drive the arm control loop. This method must be called periodically to
     * ensure the arm PID controller functions.
     */
    public void update()
    {
        double position = getAngle();

        // TODO: Debug only, move to SK22Climb
        SmartDashboard.putNumber("Arm Set Point", setPoint);
        SmartDashboard.putNumber("Arm Position", position);

        if (armEnabled)
        {
            motor.set(controller.calculate(position, setPoint));
        }
        else
        {
            motor.set(0.0);
        }
    }
}
