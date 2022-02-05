package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.SK22Transfer;
import frc.robot.utils.FilteredJoystick;


public class EjectBallCommand extends CommandBase{

    private final SK22Transfer transferSubsystem;

    private final FilteredJoystick joystickOperator;

    public EjectBallCommand(SK22Transfer transferSubsystem, FilteredJoystick joystickOperator)
    {
        this.transferSubsystem = transferSubsystem;
        this.joystickOperator = joystickOperator;

        addRequirements(transferSubsystem);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        if(joystickOperator.getRawButtonPressed(TransferConstants.ejectBallButton))
        {
            this.transferSubsystem.setExitTransferMotor(TransferConstants.BALL_EJECTION_SPEED);
            this.transferSubsystem.setIntakeTransferMotor(TransferConstants.BALL_EJECTION_SPEED);
        }
        if(joystickOperator.getRawButtonReleased(TransferConstants.ejectBallButton)){
            this.transferSubsystem.setExitTransferMotor(TransferConstants.STOP_SPEED);
            this.transferSubsystem.setIntakeTransferMotor(TransferConstants.STOP_SPEED);
        }
        

    }


    @Override
    public boolean isFinished()
    {
        return false;
    }
    
}
