package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SK22Drive;
import frc.robot.subsystems.SK22Vision;

/**
 * Uses the vision acquisition system to put the drivetrain into the correct rotation to
 * face the target. This uses
 */
public class AcquireTargetCommand extends CommandBase
{
    private final SK22Drive drive;
    private final SK22Vision vision;
    PIDController pidController;

    public AcquireTargetCommand(SK22Drive drive, SK22Vision vision)
    {
        this.drive = drive;
        this.vision = vision;
    }

    @Override
    public void initialize()
    {
        pidController = new PIDCommand(new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0),
            drive::getHeading, vision.getHorizontalAngle(), output -> drive.arcadeDrive(0, output),
            drive).getController();

        pidController.enableContinuousInput(-180, 180);
        pidController.setTolerance(DriveConstants.TURN_TOLERANCE,
            DriveConstants.TURN_RATE_TOLERANCE);

        pidController.reset();
    }

    @Override
    public void execute()
    {
        drive.arcadeDrive(0,
            pidController.calculate(drive.getHeading(), vision.getHorizontalAngle()));
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
        return pidController.atSetpoint();
    }
}
