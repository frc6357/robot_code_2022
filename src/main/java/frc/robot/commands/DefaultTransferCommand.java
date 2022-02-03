package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Transfer;

public class DefaultTransferCommand extends CommandBase{
    private final SK22Transfer transfer;
    boolean verticalFull;


    public DefaultTransferCommand(SK22Transfer transfer)
    {
        this.transfer = transfer;
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        
        
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
