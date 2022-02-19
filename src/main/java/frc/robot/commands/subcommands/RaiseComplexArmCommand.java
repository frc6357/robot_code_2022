package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Climb;

// TODO: Given that the complex arm extension will be controllable (unlike
// the simple arm), I would recommend you have a single command that merely
// sets the distance that the arm is to extend to. Whether this is a "raise"
// or a "lower" depends only on where the arm previously was and where you
// tell it to go next.

/**
 * A command to raise the motor-controlled arm in the climb mechanism.
 */
public class RaiseComplexArmCommand extends CommandBase
{
    private final SK22Climb climb;

    /**
     * Constructor for the complex arm raising command.
     * 
     * @param climb The climb subsystem on which the command operates
     * @param distance The position to which the arm must move in metres
     *                 relative to the fully retracted position. TODO: Correct?
     */
    public RaiseComplexArmCommand(SK22Climb climb, double distance)
    {
        this.climb = climb;
        addRequirements(climb);
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
