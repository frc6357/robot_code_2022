package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Launcher;

// TODO: Write this command

/**
 * A command that shoots the balls that we possess using the launcher
 * 
 */
public class ShootBallsCommand extends CommandBase
{
    private final SK22Launcher launcher;

    /**
     * Constructor for the ball shooter command.
     * 
     * @param launcher The launcher subsystem on which the command operates.
     */
    public ShootBallsCommand(SK22Launcher launcher)
    {
        this.launcher = launcher;

        addRequirements(launcher);
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
