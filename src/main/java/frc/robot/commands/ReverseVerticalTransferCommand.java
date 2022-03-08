package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.SK22Transfer;

public class ReverseVerticalTransferCommand extends CommandBase
{
    /**
     * Transfer subsystem for the LoadBallVertical Command
     */
    private final SK22Transfer transfer;

    /**
     * A manual override command that allows the operator to reverse the
     * vertical transfer belt if a ball gets jammed
     * 
     * @param transfer The transfer subsystem on which the command operates.
     */
    public ReverseVerticalTransferCommand(SK22Transfer transfer)
    {
        this.transfer = transfer;

        transfer.setVerticalTransferMotor(0.0);

        addRequirements(transfer);
    }

    /**
     * Method that sets the motor speeds to the ejection speed if the user has set the on
     * variable to true and defaults to zero if on is false
     */
    @Override
    public void initialize()
    {
        // Sets the vertical transfer motor speed
        transfer.setVerticalTransferMotor(-TransferConstants.VERTICAL_MOTOR_SPEED);
    }

    /** {@inheritDoc} */
    @Override
    public void end(boolean bInterrupted)
    {
        // Turns off the vertical transfer motor
        transfer.setVerticalTransferMotor(0.0);
    }

    /** {@inheritDoc} */
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
