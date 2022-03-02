package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22SimpleClimb;

/**
 * Sets the tilt arm to be in the neutral position
 */
public class NeutralTiltSimpleArmCommand extends CommandBase 
{
    private final SK22SimpleClimb climb;
     /**
     * Constructor for the simple arm neutral tilt command.
     * 
     * @param climb The climb subsystem on which the command operates
     */
    public NeutralTiltSimpleArmCommand(SK22SimpleClimb climb)
    {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize()
    {
        this.climb.makeTiltSimpleArmNeutral();
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
