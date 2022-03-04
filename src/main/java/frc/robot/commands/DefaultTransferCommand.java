package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Transfer;

/**
 * Command that turns all the transfer motors off
 */
public class DefaultTransferCommand extends CommandBase
{
    private final SK22Transfer transfer;

    /**
     * Constructor for the default transfer command
     * 
     * @param transfer
     *            The transfer subsystem on which this command operates.
     */
    public DefaultTransferCommand(SK22Transfer transfer)
    {
        this.transfer = transfer;

        addRequirements(transfer);
    }

    @Override
    public void initialize()
    {
        //transfer.setIntakeTransferMotor(0.0);
        //transfer.setVerticalTransferMotor(0.0);
        //transfer.setExitTransferMotor(0.0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
