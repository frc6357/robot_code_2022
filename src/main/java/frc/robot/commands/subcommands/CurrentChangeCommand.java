package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Climb;

/**
 * A command that causes the simple climb arm to be moved to the tilted position.
 */
public class CurrentChangeCommand extends CommandBase 
{

    private final SK22Climb climb;

    private double threshold;

    private boolean overCurrent = false;

    /**
     * Constuctor for the simple arm tilt command.
     * 
     * @param climb The climb subsystem on which the command operates.
     * @param threshold The current level on which we should stop the ratchet.
     */
    public CurrentChangeCommand(SK22Climb climb, double threshold)
    {
        this.climb = climb;
        this.threshold = threshold;
    }

    @Override
    public void initialize()
    {
       
    }

    @Override
    public void execute()
    {
        if (climb.getMotorCurrent() >= threshold)
        {
            overCurrent = true;
        }
    }

    @Override
    public boolean isFinished()
    {
        return overCurrent;
    }

    
}
