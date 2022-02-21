package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.subcommands.LowerSimpleArmCommand;
import frc.robot.commands.subcommands.MoveComplexArmLiftCommand;
import frc.robot.commands.subcommands.PivotComplexArmCommand;
import frc.robot.commands.subcommands.RaiseSimpleArmCommand;
import frc.robot.commands.subcommands.TiltSimpleArmCommand;
import frc.robot.commands.subcommands.TimeDelayCommand;
import frc.robot.subsystems.SK22Climb;

public final class ClimbSequence
{

    public static Command getStep1(SK22Climb climb)
    {
        ParallelCommandGroup step1 = new ParallelCommandGroup();
        step1.addCommands(new RaiseSimpleArmCommand(climb));
        step1.addCommands(new PivotComplexArmCommand(climb, ClimbConstants.COMPLEX_FULL_TILT));
        return step1;
    }

    public static Command getStep2(SK22Climb climb)
    {
        ParallelCommandGroup step2 = new ParallelCommandGroup();
        step2.addCommands(new LowerSimpleArmCommand(climb));
        step2.addCommands(new MoveComplexArmLiftCommand(climb, ClimbConstants.COMPLEX_FULL_EXTEND));
        return step2;
    }

    public static Command getStep3(SK22Climb climb)
    {
        return new PivotComplexArmCommand(climb, ClimbConstants.COMPLEX_PARTIAL_STRAIGHTEN);
    }

    //TODO: Need command for step 4 (retract to current change)
    //TODO: change simple Climb arm to single solenoids(need neutral state)
    public static Command getStep6(SK22Climb climb)
    {
        ParallelCommandGroup step6 = new ParallelCommandGroup();
        step6.addCommands(new RaiseSimpleArmCommand(climb));
        step6.addCommands(new TimeDelayCommand(ClimbConstants.STEP6_DELAY_MILLIS));
        return step6;
    }

    public static Command getStep7(SK22Climb climb)
    {
        ParallelCommandGroup step7 = new ParallelCommandGroup();
        step7.addCommands(new TiltSimpleArmCommand(climb));
        step7.addCommands(new TimeDelayCommand(ClimbConstants.STEP7_DELAY_MILLIS));
        return step7;
    }

    public static Command getStep8(SK22Climb climb)
    {
        ParallelCommandGroup step8 = new ParallelCommandGroup();
        step8.addCommands(new LowerSimpleArmCommand(climb));
        step8.addCommands(new TimeDelayCommand(ClimbConstants.STEP8_DELAY_MILLIS));
        return step8;
    }

    //TODO:step 9 needs neutral stage
    public static Command getStep10(SK22Climb climb)
    {
        ParallelCommandGroup step10 = new ParallelCommandGroup();
        step10.addCommands(new LowerSimpleArmCommand(climb));
        step10.addCommands(new TimeDelayCommand(ClimbConstants.STEP10_DELAY_MILLIS));
        return step10;
    }

    public static Command getStep11(SK22Climb climb)
    {
        return new MoveComplexArmLiftCommand(climb, ClimbConstants.COMPLEX_PARTIAL_EXTEND);
    }

    public static Command getStep12(SK22Climb climb)
    {
        return new PivotComplexArmCommand(climb, ClimbConstants.COMPLEX_FULL_TILT);
    }
}
