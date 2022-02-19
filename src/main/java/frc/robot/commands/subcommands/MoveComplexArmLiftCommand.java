package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Climb;


/**
 * Command that allows the ratchet to a set distance
 */
public class MoveComplexArmLiftCommand extends CommandBase
{
    private final SK22Climb climb;
    private double distance;

    /**
     * The constructor instantiates all the member variables for the Move Complex arm lift command
     * @param climb The SK22Climb subsytem
     * @param distance The amount of distance the ratchet would cover
     */
    public MoveComplexArmLiftCommand(SK22Climb climb, double distance)
    {
        this.climb = climb;
        this.distance = distance;
    }

    @Override
    public void initialize()
    {
        this.climb.setComplexArm(this.distance);
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
