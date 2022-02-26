package frc.robot.commands;

import javax.swing.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SK22Transfer;
import frc.robot.utils.TimerType;

public class DefaultTransferCommand extends CommandBase{
    private final SK22Transfer transfer;
    boolean verticalFull;

    private int timerElapsed = 0;

    private TimerType timerType;

    // Just for testing
    private boolean testVerticalFull = false;

    public DefaultTransferCommand(SK22Transfer transfer)
    {
        this.transfer = transfer;

        addRequirements(transfer);
    }

    @Override
    public void initialize()
    {
        
    }
    

    @Override
    public void execute()
    {
        // Position one   = the spot where the intake meets the transfer
        // Position two   = the ejection port
        // Position three = the vertical shaft hold

        // This detects if we have just received a ball from the intake
        if(transfer.getPositionOnePresence())
        {
            // If the ball is the correct color
            if(transfer.getColorSensor().equals(transfer.teamColor))
            {
                // Handle storing the ball & locking the intake so we cannot 
                // intake any more balls while we are processing the one we 
                // already have in the system

                // Replace the conditions with:
                // !transfer.getPositionThreePresence()
                if(testVerticalFull)
                {
                    // Transfer ball to the vertical hold
                    timerType = TimerType.VERTICAL;
                    transfer.setTimerState(true);

                    transfer.setIntakeTransferMotor(Constants.TransferConstants.INTAKE_MOTOR_SPEED);
                    transfer.setExitTransferMotor(-Constants.TransferConstants.EXIT_MOTOR_SPEED);
                } 
                else if(testVerticalFull && transfer.getIsRunningTimerEnabled())
                {
                    // Replace above if's conditions with:
                    // transfer.getPositionThreePresence() && transfer.getIsRunningTimerEnabled()

                    transfer.setTimerState(false);
                    updateTimer();
                }
                // If there is a ball in the vertical hold, than we dont need to do anything
                // with the ball in the horizontal hold because it's in a good spot
            }
            else 
            {
                timerType = TimerType.EJECT;
                transfer.setTimerState(true);

                transfer.setIntakeTransferMotor(Constants.TransferConstants.INTAKE_MOTOR_SPEED);
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

    private void updateTimer()
    {
        if(timerType == TimerType.VERTICAL) 
        {
            if(timerElapsed < Constants.TransferConstants.TRANSFER_TO_VERTICAL_SHAFT_DURATION && 
            transfer.getIsRunningTimerEnabled()) 
            {
                timerElapsed++;
            } 
            else if(timerElapsed >= Constants.TransferConstants.TRANSFER_TO_VERTICAL_SHAFT_DURATION)
            {
                timerElapsed = 0;
                transfer.setTimerState(false);

                transfer.setIntakeTransferMotor(0);
                transfer.setExitTransferMotor(0);
            } else if(!transfer.getIsRunningTimerEnabled() && timerElapsed > 0)
            {
                timerElapsed = 0;

                transfer.setIntakeTransferMotor(0);
                transfer.setExitTransferMotor(0);
            }
        }
        else
        {
            if(timerElapsed < Constants.TransferConstants.EJECT_DURATION && 
            transfer.getIsRunningTimerEnabled()) 
            {
                timerElapsed++;
            } 
            else if(timerElapsed >= Constants.TransferConstants.EJECT_DURATION)
            {
                timerElapsed = 0;
                transfer.setTimerState(false);

                transfer.setIntakeTransferMotor(0);
                transfer.setExitTransferMotor(0);
            } else if(!transfer.getIsRunningTimerEnabled() && timerElapsed > 0)
            {
                timerElapsed = 0;

                transfer.setIntakeTransferMotor(0);
                transfer.setExitTransferMotor(0);
            }
        }
    }
}
