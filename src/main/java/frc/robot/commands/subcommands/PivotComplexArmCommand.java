package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Climb;

/**
 * Command that allows the complex arm to be pivoted to any position from 0-360 degrees
 */
public class PivotComplexArmCommand extends CommandBase
{
    private final SK22Climb climb;
    private double degrees;

    /**
     * Constructor instantiates all the member variables for the pivot complex arm command
     * @param climb The SK22Climb subsystem
     * @param degrees The position we want to move the arm to
     */
    public PivotComplexArmCommand(SK22Climb climb, double degrees)
    {
        this.climb = climb;
        this.degrees = degrees;
    }

    @Override
    public void initialize()
    {
        this.climb.setComplexArm(this.degrees);
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
