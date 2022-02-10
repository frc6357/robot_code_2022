package frc.robot.commands.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SK22Climb;
import frc.robot.subsystems.SK22Intake;
import frc.robot.subsystems.SK22SimpleClimbArm;


public class RetractSimpleArmCommand extends CommandBase{
    private final SK22SimpleClimbArm simpleClimbArm;

    public RetractSimpleArmCommand(SK22SimpleClimbArm simpleClimbArm)
    {
        this.simpleClimbArm = simpleClimbArm;
    }

    @Override
    public void initialize()
    {
        this.simpleClimbArm.retractHand();
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
