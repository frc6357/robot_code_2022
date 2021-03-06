package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22SimpleClimb;

/**
 * Sets the extension arms to be in the neutral position
 */
public class NeutralRaiseSimpleArmCommand extends CommandBase
{
    private final SK22SimpleClimb climb;

    /**
     * Constructor for the simple arm neutral tilt command.
     * 
     * @param climb
     *            The climb subsystem on which the command operates
     */
    public NeutralRaiseSimpleArmCommand(SK22SimpleClimb climb)
    {

        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize()
    {
        this.climb.makeRaiseSimpleArmNeutral();
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
