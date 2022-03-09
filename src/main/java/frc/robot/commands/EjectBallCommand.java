package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.SK22Launcher;
import frc.robot.subsystems.SK22Transfer;

/**
 * A command that ejects balls through the horizontal exit in
 * case of an emergency.
 */
public class EjectBallCommand extends CommandBase
{

    /**
     * Transfer Subsystem for the ejectball command
     */
    private final SK22Transfer transfer;

    private final SK22Launcher launcher;

    /**
     * Manual override command allowing the operator to eject any ball
     * currently in the horizontal portion of the transfer.
     * 
     * @param transfer The transfer subsystem the command operates on.
     * @param launcher The launcher subsystem the command operates on.
     */
    public EjectBallCommand(SK22Transfer transfer, SK22Launcher launcher)
    {
        this.transfer = transfer;
        this.launcher = launcher;

        addRequirements(transfer);
    }

    /**
     * Method that sets the motor speeds to the ejection speed if the user has set the on
     * variable to true and defaults to zero if on is false
     */
    @Override
    public void initialize()
    {
        //Sets the exist transfer motor to ejectionspeed
        launcher.setExitTransferMotor(-TransferConstants.BALL_EJECTION_SPEED);
        //Sets the intake transfer motor to ejectionspeed
        transfer.setIntakeTransferMotor(TransferConstants.BALL_EJECTION_SPEED);
    }
    
    /** {@inheritDoc} */
    @Override
    public void end(boolean interrupted)
    {
        // Turns off the exit transfer motor 
        launcher.setExitTransferMotor(0.0);
        // Turns off the intake transfer motor
        transfer.setIntakeTransferMotor(0.0);
    }

    /** {@inheritdoc} */
    @Override
    public boolean isFinished()
    {
        return false;
    }

}
