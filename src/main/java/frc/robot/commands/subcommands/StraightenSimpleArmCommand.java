package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Climb;

/**
 * A command that causes the simple climb arm to be moved to the straight position.
 */
public class StraightenSimpleArmCommand extends CommandBase
{

    private final SK22Climb climb;

    /**
     * Constuctor for the simple arm straighten command.
     * 
     * @param climb The climb subsystem on which the command operates.
     */
    public StraightenSimpleArmCommand(SK22Climb climb)
    {
        this.climb = climb;
    }

    @Override
    public void initialize()
    {
        this.climb.straighten();
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
