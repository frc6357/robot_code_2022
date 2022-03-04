package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.SK22Launcher;

/**
 * Sets the launcher to a specified speed and ends when the launcher
 * reaches the said speed.
 */
public class SetLauncherSpeedCommand extends CommandBase
{
    private final SK22Launcher launcher;

    /**
     * Constructor for the ball shooter command.
     * 
     * @param launcher
     *            The launcher subsystem on which the command operates.
     */
    public SetLauncherSpeedCommand(SK22Launcher launcher)
    {
        this.launcher = launcher;

        addRequirements(launcher);
    }

    @Override
    public void initialize()
    {
        launcher.setLauncherRPM(AutoConstants.AUTO_LAUNCH_SPEED);
        launcher.enableLauncher();
    }

    @Override
    public void end(boolean interrupted)
    {
        if (interrupted)
        {
            launcher.disableLauncher();
        }
    }

    @Override
    public boolean isFinished()
    {
        // Check if difference between setpoint and actual rpm is within tolerance
        return Math.abs(launcher.getLauncherRPMSetpoint() - launcher.getLauncherRPM())
            <= LauncherConstants.LAUNCHER_RPM_ERROR_TOLERANCE;
    }
}
