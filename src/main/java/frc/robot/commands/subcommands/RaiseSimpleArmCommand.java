package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22Climb;
import frc.robot.subsystems.base.SimpleClimbArm;

public class RaiseSimpleArmCommand extends CommandBase
{
    private final SK22Climb climb;

    public RaiseSimpleArmCommand(SK22Climb climb)
    {
        this.climb = climb;
    }

    @Override
    public void initialize()
    {
        this.climb.raise();
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
