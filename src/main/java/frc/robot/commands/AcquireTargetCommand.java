package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SK22Drive;
import frc.robot.subsystems.SK22Vision;

/**
 * Uses the vision acquisition system to put the drivetrain into the correct rotation to
 * face the target. This uses
 */
public class AcquireTargetCommand extends CommandBase
{
    private final SK22Drive  drive;
    private final SK22Vision vision;
    private PIDController    pidController;

    /** Desired angle of the drivetrain */
    private double setpoint = 0.0;

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

        pidController = new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0);

        pidController.enableContinuousInput(-180, 180);
        pidController.setTolerance(DriveConstants.TURN_TOLERANCE,
            DriveConstants.TURN_RATE_TOLERANCE);

        addRequirements(drive, vision);
    }

    @Override
    public void initialize()
    {
        setpoint = drive.getHeading() + vision.getHorizontalAngle().get();
        pidController.reset();
    }

    @Override
    public void execute()
    {
        drive.arcadeDrive(0, pidController.calculate(drive.getHeading(), setpoint));
    }

    @Override
    public void end(boolean interrupted)
    {
        drive.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished()
    {
        // End when the controller is at the reference.
        return (pidController.atSetpoint());
    }
}
