package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SK22Launcher;

/**
 * A command that shoots the balls that we possess using the launcher. This is intended to
 * be used with the button's whenHeld() method. It starts the launcher transfer motor and
 * keeps it running as long as the button is held. When the button is released, the motor
 * is stopped.
 */
public class ShootBallsCommand extends CommandBase
{
    private final SK22Launcher launcher;

    /**
     * Constructor for the ball shooter command.
     * 
     * @param launcher
     *            The launcher subsystem on which the command operates.
     */
    public ShootBallsCommand(SK22Launcher launcher)
    {
        this.launcher = launcher;

        addRequirements(launcher);
    }

    @Override
    public void initialize()
    {
        // Turn on the launcher transfer motor.
        launcher.setLauncherTransferMotor(Constants.LauncherConstants.LAUNCHER_TRANSFER_SPEED);
        // TODO: Remove this; Testing launcher direction only
        launcher.setLauncherRPM(6000);
        launcher.enableLauncher();
    }

    @Override
    public void execute()
    {
        // Nothing needed here
    }

    @Override
    public void end(boolean bInterrupted)
    {
        // Turn on the launcher transfer motor.
        launcher.setLauncherTransferMotor(0.0);
        launcher.disableLauncher();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
