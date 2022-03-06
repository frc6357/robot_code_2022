package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SK22Launcher;
import frc.robot.subsystems.SK22Transfer;

/**
 * A command that shoots the balls that we possess using the launcher. This is intended to
 * be used with the button's whenHeld() method. It starts the launcher transfer motor, vertical
 * transfer motor, transfer exit motor and keeps it running as long as the button is held. 
 * When the button is released, the motors are stopped.
 */
public class ShootBallsCommand extends CommandBase
{
    private final SK22Launcher launcher;
    private final SK22Transfer transfer;

    /**
     * Constructor for the ball shooter command.
     * 
     * @param launcher
     *            The launcher subsystem on which the command operates.
     * @param transfer
     *            The transfer subsystem on which the command operates.
     */
    public ShootBallsCommand(SK22Launcher launcher, SK22Transfer transfer)
    {
        this.launcher = launcher;
        this.transfer = transfer;

        addRequirements(launcher);
        addRequirements(transfer);
    }

    @Override
    public void initialize()
    {
        // Turn on the launcher transfer motor.
        launcher.setLauncherTransferMotor(Constants.LauncherConstants.LAUNCHER_TRANSFER_SPEED);
        // Turn on the transfer exit motor.
        transfer.setExitTransferMotor(Constants.TransferConstants.VERTICAL_MOTOR_SPEED);
        // Turn on the vertical transfer motor.
        transfer.setVerticalTransferMotor(Constants.TransferConstants.LOAD_BALL_VERTICAL_SPEED);
        // Turn ont the transfer intake motor.
        transfer.setIntakeTransferMotor(Constants.TransferConstants.INTAKE_MOTOR_SPEED);
    }

    @Override
    public void execute()
    {
        // Nothing needed here
    }

    @Override
    public void end(boolean bInterrupted)
    {
        // Turn off the launcher transfer motor.
        launcher.setLauncherTransferMotor(0.0);
        // Turn off the transfer exit motor.
        transfer.setExitTransferMotor(0.0);
        // Turn off the vertical transfer motor.
        transfer.setVerticalTransferMotor(0.0);
        // Turn off the transfer intake motor.
        transfer.setIntakeTransferMotor(0.0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
