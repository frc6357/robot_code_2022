package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22SimpleClimbArm;

public class ExtendSimpleArmCommand extends CommandBase
{
    private final SK22SimpleClimbArm simpleClimbArm;

    public ExtendSimpleArmCommand(SK22SimpleClimbArm simpleClimbArm)
    {
        this.simpleClimbArm = simpleClimbArm;
    }

    @Override
    public void initialize()
    {
        this.simpleClimbArm.extendHand();
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
