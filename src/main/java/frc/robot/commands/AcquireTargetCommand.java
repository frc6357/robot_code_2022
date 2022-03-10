package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SK22Drive;
import frc.robot.subsystems.SK22Vision;

/**
 * Uses the vision acquisition system to put the drivetrain into the correct rotation to
 * face the target. This uses a PID controller to set the speeds of the drivetrain.
 */
public class AcquireTargetCommand extends CommandBase
{
    private final SK22Drive  drive;
    private final SK22Vision vision;
    private PIDController    pidController;

    /** Desired angle of the drivetrain */
    private double setpoint = 0.0;
    
    private boolean validSetpoint = false;

    /**
     * Creates a new AcquireTargetCommand and sets the
     * 
     * @param drive
     *            The drive subsystem required to move the robot
     * @param vision
     *            The vision subsystem required to find the position of the robot relative
     *            to the goal
     */
    public AcquireTargetCommand(SK22Drive drive, SK22Vision vision)
    {
        this.drive = drive;
        this.vision = vision;

        // TODO: Need to tune this value so as to not destroy the robot when turning
        pidController = new PIDController(DriveConstants.KP_TURN_DEGREES, 0, 0);

        pidController.enableContinuousInput(-180, 180);
        // TODO: Need to tune constraints as well to make sure we don't overshoot or undershoot
        pidController.setTolerance(DriveConstants.TURN_TOLERANCE,
            DriveConstants.TURN_RATE_TOLERANCE);

        addRequirements(drive, vision);
    }

    @Override
    public void initialize()
    {
        validSetpoint = false;
        pidController.reset();
    }

    @Override
    public void execute()
    {
        if (vision.isTargetInFrame())
        {
            setpoint = drive.getHeading() + vision.getHorizontalAngle().get();
            validSetpoint = true;

            drive.arcadeDrive(0, pidController.calculate(vision.getHorizontalAngle().get(), 0));
        }
        
        if (validSetpoint)
        {
            drive.arcadeDrive(0, pidController.calculate(drive.getHeading(), setpoint));
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        drive.arcadeDrive(0, 0);
        validSetpoint = false;
    }

    @Override
    public boolean isFinished()
    {
        // End when the controller is at the reference or drivetrain is within tolerance
        return ((Math.abs(drive.getHeading() - setpoint)
                <= DriveConstants.ACQUIRE_TARGET_TOLERANCE))
            || pidController.atSetpoint();
    }
}
