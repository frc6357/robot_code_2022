package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ports;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.SK22Transfer;
import frc.robot.utils.FilteredJoystick;


public class EjectBallCommand extends CommandBase{

    private final SK22Transfer transferSubsystem;
    private boolean on;
 

    public EjectBallCommand(SK22Transfer transferSubsystem, boolean on)
    {
        this.transferSubsystem = transferSubsystem;
        this.on = on;

        addRequirements(transferSubsystem);
    }

    @Override
    public void initialize()
    {
        this.transferSubsystem.setExitTransferMotor(on? TransferConstants.BALL_EJECTION_SPEED : 0);
        this.transferSubsystem.setIntakeTransferMotor(on? TransferConstants.BALL_EJECTION_SPEED: 0);
    }

    @Override
    public void execute()
    {
        
    }


    @Override
    public boolean isFinished()
    {
        return true;
    }
    
}
