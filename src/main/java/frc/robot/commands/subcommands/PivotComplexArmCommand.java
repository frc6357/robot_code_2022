package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22ComplexClimb;
//TODO: This is currently not needed but could be useful in the future.
/**
 * Command that allows the complex arm to be pivoted to any position from 0-360 degrees
 */
public class PivotComplexArmCommand extends CommandBase
{
    private final SK22ComplexClimb climb;
    private double degrees;

    /**
     * Constructor instantiates all the member variables for the pivot complex arm command
     * @param climb The SK22ComplexClimb subsystem
     * @param degrees The position we want to move the arm to
     */
    public PivotComplexArmCommand(SK22ComplexClimb climb, double degrees)
    {
        this.climb = climb;
        this.degrees = degrees;
    }

    @Override
    public void initialize()
    {
        this.climb.setComplexPivotPosition(this.degrees);
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
