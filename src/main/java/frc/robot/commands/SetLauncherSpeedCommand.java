package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Launcher;

/**
 * Sets the launcher to a specified speed and ends when the launcher reaches the said
 * speed.
 */
public class SetLauncherSpeedCommand extends CommandBase
{
    private final SK22Launcher launcher;
    private final double       speed;

    /**
     * Constructor for the ball shooter command.
     * 
     * @param launcher
     *            The launcher subsystem on which the command operates.
     * @param speed
     *            The launcher speed to be set.
     */
    public SetLauncherSpeedCommand(SK22Launcher launcher, double speed)
    {
        this.launcher = launcher;
        this.speed = speed;

        addRequirements(launcher);
    }

    @Override
    public void initialize()
    {
        launcher.setLauncherRPM(speed);
    }

    @Override
    public void end(boolean interrupted)
    {

    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
