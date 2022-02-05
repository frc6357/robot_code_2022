package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.SK22Transfer;


public class EjectBallCommand extends CommandBase{

    private final SK22Transfer transfer;


    public EjectBallCommand(SK22Transfer transfer)
    {
        this.transfer = transfer;
    }

    @Override
    public void initialize()
    {
        if(this.transfer.getPositionTwoPrecense() || this.transfer.getPositionThreePrecense())
        {
            this.transfer.setPositionThreeMotor(TransferConstants.BALL_EJECTION_SPEED);
            this.transfer.setPositionOneMotor(TransferConstants.BALL_EJECTION_SPEED);
        }
        else
        {
            this.transfer.setPositionOneMotor(TransferConstants.STOP_SPEED);
            this.transfer.setPositionThreeMotor(TransferConstants.STOP_SPEED);
        }
    }


    @Override
    public boolean isFinished()
    {
        return false;
    }
    
}
