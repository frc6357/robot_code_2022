package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22ComplexClimb;


/**
 * Command that allows the ratchet to a set distance
 */
public class RotateComplexArmCommand extends CommandBase
{
    private final SK22ComplexClimb climb;
    private boolean direction;

    /**
     * The constructor instantiates all the member variables for the Move Complex arm lift command
     * @param climb The SK22Climb subsytem
     * @param direction The direction in which the complex structure moves 
     */
    public RotateComplexArmCommand(SK22ComplexClimb climb, boolean direction)
    {
        this.climb = climb;
        this.direction = direction;
    }

    @Override
    public void initialize()
    {
        this.climb.rotateComplexArm(this.direction);
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
