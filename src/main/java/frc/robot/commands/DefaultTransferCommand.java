package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SK22Transfer;
import frc.robot.utils.TimerType;

/**
 * Command to automatically manage moving balls through the transfer subsystem.
 */
public class DefaultTransferCommand extends CommandBase
{
    private final SK22Transfer transfer;
    private boolean            verticalFull;

    private int timerElapsed = 0;

    private TimerType timerType;

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
        transfer.setIntakeTransferMotor(-1.0);

        // TODO: just for testing
        verticalFull = false;
    }

    @Override
    public void execute()
    {
        // Position one = the spot where the intake meets the transfer
        // Position two = the ejection port
        // Position three = the vertical shaft hold

        System.out.println(transfer.getColorSensor());

        // This detects if we have just received a ball from the intake
        if (transfer.getPositionOnePresence())
        {
            System.out.println("Presence detected in position one");
            // If the ball is the correct color
            if (transfer.getColorSensor().equals(transfer.teamColor))
            {
                // Handle storing the ball & locking the intake so we cannot
                // intake any more balls while we are processing the one we
                // already have in the system
                // TODO: change this to "!transfer.getPositionThreePresence()" when done testing
                if (!verticalFull)
                {
                    // Transfer ball to the vertical hold
                    timerType = TimerType.VERTICAL;
                    transfer.setIsRunningTimerEnabled(true);

                    transfer.setExitTransferMotor(-Constants.TransferConstants.EXIT_MOTOR_SPEED);
                }
                else if (verticalFull && transfer.getIsRunningTimerEnabled())
                {
                    transfer.setIsRunningTimerEnabled(false);
                    updateTimer();
                }
                // TODO: Create a case for when there is a ball in the launcher system and we
                // want to keep the ball that we just picked up
            }
            else
            {
                System.out.println("Wrong color, ejecting!");
                timerType = TimerType.EJECT;
                transfer.setIsRunningTimerEnabled(true);

                transfer.setExitTransferMotor(Constants.TransferConstants.EXIT_MOTOR_SPEED);
            }
        }

        updateTimer();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    /**
     * Counts method calls for timer
     * <p>
     * VERTICAL timer controls the motors required for shifting balls into the vertical
     * shaft EJECT timer controls the motors required for ejecting the ball out the back
     */
    private void updateTimer()
    {
        if (timerType == TimerType.VERTICAL)
        {
            if (timerElapsed < Constants.TransferConstants.TRANSFER_TO_VERTICAL_SHAFT_DURATION
                && transfer.getIsRunningTimerEnabled())
            {
                timerElapsed++;
            }
            else if (timerElapsed >= Constants.TransferConstants.TRANSFER_TO_VERTICAL_SHAFT_DURATION)
            {
                timerElapsed = 0;
                transfer.setIsRunningTimerEnabled(false);

                transfer.setExitTransferMotor(0);
            }
            else if (!transfer.getIsRunningTimerEnabled() && timerElapsed > 0)
            {
                timerElapsed = 0;

                transfer.setExitTransferMotor(0);
            }
        }
        else
        {
            if (timerElapsed < Constants.TransferConstants.EJECT_DURATION
                && transfer.getIsRunningTimerEnabled())
            {
                timerElapsed++;
            }
            else if (timerElapsed >= Constants.TransferConstants.EJECT_DURATION)
            {
                timerElapsed = 0;
                transfer.setIsRunningTimerEnabled(false);

                transfer.setExitTransferMotor(0);
            }
            else if (!transfer.getIsRunningTimerEnabled() && timerElapsed > 0)
            {
                timerElapsed = 0;

                transfer.setExitTransferMotor(0);
            }
        }
    }
}
