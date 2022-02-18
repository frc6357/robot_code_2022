package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Climb;

public class RaiseComplexArmCommand extends CommandBase{

    private final SK22Climb climb;

    public RaiseComplexArmCommand(SK22Climb climb, double distance)
    {
        this.climb = climb;
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
        return true;
    }

    
}
