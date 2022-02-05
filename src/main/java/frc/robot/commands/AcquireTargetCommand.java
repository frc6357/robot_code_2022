package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Drive;
import frc.robot.subsystems.SK22Vision;

/**
 * Uses the vision acquisition system to put the drivetrain
 * into the correct rotation to face the target.
 */
// TODO: Write this command
public class AcquireTargetCommand extends CommandBase
{
    private final SK22Drive drive;
    private final SK22Vision vision;

    public AcquireTargetCommand(SK22Drive drive, SK22Vision vision)
    {
        this.drive = drive;
        this.vision = vision;
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
