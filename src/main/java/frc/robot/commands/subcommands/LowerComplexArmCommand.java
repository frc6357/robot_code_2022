package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Climb;

/**
 * A command to lower the motor-controlled arm in the climb mechanism.
 */
class LowerComplexArmCommand extends CommandBase
{
    private final SK22Climb climb;

    /**
     * Constructor for the complex arm lowering command.
     * 
     * @param climb The climb subsystem on which the command operates
     */
    public LowerComplexArmCommand(SK22Climb climb)
    {
        this.climb = climb;
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
