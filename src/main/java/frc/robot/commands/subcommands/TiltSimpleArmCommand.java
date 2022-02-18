package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Climb;

/**
 * A command that causes the simple climb arm to be moved to the tilted position.
 */
public class TiltSimpleArmCommand extends CommandBase 
{

    private final SK22Climb climb;

    /**
     * Constuctor for the simple arm tilt command.
     * 
     * @param climb The climb subsystem on which the command operates.
     */
    public TiltSimpleArmCommand(SK22Climb climb)
    {
        this.climb = climb;
    }

    @Override
    public void initialize()
    {
        this.climb.tilt();
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
