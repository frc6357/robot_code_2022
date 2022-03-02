package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22SimpleClimb;

/**
 * A command to raise the pneumatically-controlled arm in the climb mechanism.
 */
public class RaiseSimpleArmCommand extends CommandBase
{
    private final SK22SimpleClimb climb;

    /**
     * Constructor for the simple arm raising command.
     * 
     * @param climb The climb subsystem on which the command operates
     */
    public RaiseSimpleArmCommand(SK22SimpleClimb climb)
    {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize()
    {
        this.climb.raiseSimpleArm();
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
