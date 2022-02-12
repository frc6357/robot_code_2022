package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Climb;

public class TiltSimpleArmCommand extends CommandBase{

    private final SK22Climb climb;

    public TiltSimpleArmCommand(SK22Climb climb)
    {
        this.climb = climb;
    }

    @Override
    public void initialize()
    {
        this.climb.tilt();
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
