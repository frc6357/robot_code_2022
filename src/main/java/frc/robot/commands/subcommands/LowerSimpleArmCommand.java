package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22SimpleClimb;

/**
 * A command to lower the pneumatically-controlled arm in the climb mechanism.
 */
public class LowerSimpleArmCommand extends CommandBase
{
    private final SK22SimpleClimb climb;

    /**
     * Constructor for the simple arm lowering command.
     * 
     * @param climb The climb subsystem on which the command operates
     */
    public LowerSimpleArmCommand(SK22SimpleClimb climb)
    {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize()
    {
        this.climb.lowerSimpleArm();
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
