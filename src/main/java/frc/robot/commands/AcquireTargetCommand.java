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
    
    /** Whether valid hori angle has been received */
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

        // Taken from characterization
        pidController = new PIDController(
            DriveConstants.KP_TURN_DEGREES,
            DriveConstants.KI_TURN_DEGREES,
            DriveConstants.KD_TURN_DEGREES);

        // Taken from SysID values used to calculate PID
        pidController.setTolerance(
            DriveConstants.TURN_TOLERANCE,
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
        // Calculates arcade drive values if vision has sent horizontal angle
        if (validSetpoint)
        {
            drive.arcadeDrive(0, -pidController.calculate(drive.getHeading(), setpoint));
        }
        // Checks if the vision target has a valid angle if valid angle
            // has not already been given
        else if (vision.isTargetInFrame())
        {
            // Calculates the desired final angle
            setpoint = drive.getHeading() + vision.getHorizontalAngle().get();
            pidController.setSetpoint(setpoint);
            validSetpoint = true;
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        // Turns off drivetrain motors once finished
        drive.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished()
    {
        // End when the controller is at the reference.
        return (pidController.atSetpoint() && validSetpoint);
    }
}
