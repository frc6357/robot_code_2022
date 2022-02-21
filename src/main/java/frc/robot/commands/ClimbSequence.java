package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.subcommands.LowerSimpleArmCommand;
import frc.robot.commands.subcommands.MoveComplexArmLiftCommand;
import frc.robot.commands.subcommands.NeutralRaiseSimpleArmCommand;
import frc.robot.commands.subcommands.NeutralTiltSimpleArmCommand;
import frc.robot.commands.subcommands.PivotComplexArmCommand;
import frc.robot.commands.subcommands.RaiseSimpleArmCommand;
import frc.robot.commands.subcommands.StraightenSimpleArmCommand;
import frc.robot.commands.subcommands.TiltSimpleArmCommand;
import frc.robot.commands.subcommands.TimeDelayCommand;
import frc.robot.subsystems.SK22Climb;

public final class ClimbSequence
{
    private ClimbSequence()
    {
        // Do nothing here
    }

    /**
     * Command sequence for climb step 1 
     * @param climb SK22Climb Subsystem
     * @return The command containing the actions for step 1 
     */
    public static Command getStep1(SK22Climb climb)
    {
        ParallelCommandGroup step1 = new ParallelCommandGroup();
        step1.addCommands(new RaiseSimpleArmCommand(climb));
        step1.addCommands(new PivotComplexArmCommand(climb, ClimbConstants.COMPLEX_FULL_TILT));
        return step1;
    }

    /**
     * Command sequence for climb step 2
     * @param climb SK22Climb Subsystem
     * @return The command containing the actions for step 2
     */
    public static Command getStep2(SK22Climb climb)
    {
        ParallelCommandGroup step2 = new ParallelCommandGroup();
        step2.addCommands(new LowerSimpleArmCommand(climb));
        step2.addCommands(new MoveComplexArmLiftCommand(climb, ClimbConstants.COMPLEX_FULL_EXTEND));
        return step2;
    }

    /**
     * Command sequence for climb step 3
     * @param climb SK22Climb Subsystem
     * @return The command containing the actions for step 3
     */
    public static Command getStep3(SK22Climb climb)
    {
        return new PivotComplexArmCommand(climb, ClimbConstants.COMPLEX_PARTIAL_STRAIGHTEN);
    }

    //TODO: Need command for step 4 (retract to current change)

    /**
     * Command sequence for climb step 5
     * @param climb SK22Climb Subsystem
     * @return The command containing the actions for step 5
     */
    public static Command getStep5(SK22Climb climb)
    {
        ParallelCommandGroup step5 = new ParallelCommandGroup();
        step5.addCommands(new NeutralTiltSimpleArmCommand(climb));
        step5.addCommands(new NeutralRaiseSimpleArmCommand(climb));
        step5
            .addCommands(new MoveComplexArmLiftCommand(climb, ClimbConstants.COMPLEX_FULL_RETRACT));
        step5
            .addCommands(new PivotComplexArmCommand(climb, ClimbConstants.COMPLEX_FULL_STRAIGHTEN));
        return step5;
    }

    /**
     * Command sequence for climb step 6
     * @param climb SK22Climb Subsystem
     * @return The command containing the actions for step 6
     */
    public static Command getStep6(SK22Climb climb)
    {
        ParallelCommandGroup step6 = new ParallelCommandGroup();
        step6.addCommands(new RaiseSimpleArmCommand(climb));
        step6.addCommands(new TimeDelayCommand(ClimbConstants.STEP6_DELAY_MILLIS));
        return step6;
    }

    /**
     * Command sequence for climb step 7
     * @param climb SK22Climb Subsystem
     * @return The command containing the actions for step 7
     */
    public static Command getStep7(SK22Climb climb)
    {
        ParallelCommandGroup step7 = new ParallelCommandGroup();
        step7.addCommands(new TiltSimpleArmCommand(climb));
        step7.addCommands(new TimeDelayCommand(ClimbConstants.STEP7_DELAY_MILLIS));
        return step7;
    }

    /**
     * Command sequence for climb step 8
     * @param climb SK22Climb Subsystem
     * @return The command containing the actions for step 8
     */
    public static Command getStep8(SK22Climb climb)
    {
        ParallelCommandGroup step8 = new ParallelCommandGroup();
        step8.addCommands(new LowerSimpleArmCommand(climb));
        step8.addCommands(new TimeDelayCommand(ClimbConstants.STEP8_DELAY_MILLIS));
        return step8;
    }

    /**
     * Command sequence for climb step 9
     * @param climb SK22Climb Subsystem
     * @return The command containing the actions for step 9
     */
    public static Command getStep9(SK22Climb climb)
    {
        ParallelCommandGroup step9 = new ParallelCommandGroup();
        step9.addCommands(new NeutralRaiseSimpleArmCommand(climb));
        step9.addCommands(new StraightenSimpleArmCommand(climb));
        return step9;
    }

    /**
     * Command sequence for climb step 10
     * @param climb SK22Climb Subsystem
     * @return The command containing the actions for step 10
     */
    public static Command getStep10(SK22Climb climb)
    {
        ParallelCommandGroup step10 = new ParallelCommandGroup();
        step10.addCommands(new LowerSimpleArmCommand(climb));
        step10.addCommands(new TimeDelayCommand(ClimbConstants.STEP10_DELAY_MILLIS));
        return step10;
    }

    /**
     * Command sequence for climb step 11
     * @param climb SK22Climb Subsystem
     * @return The command containing the action for step 11
     */
    public static Command getStep11(SK22Climb climb)
    {
        return new MoveComplexArmLiftCommand(climb, ClimbConstants.COMPLEX_PARTIAL_EXTEND);
    }

    /**
     * Command sequence for climb step 12
     * @param climb SK22Climb Subsystem
     * @return The command containing the action for step 12
     */
    public static Command getStep12(SK22Climb climb)
    {
        return new PivotComplexArmCommand(climb, ClimbConstants.COMPLEX_FULL_TILT);
    }
    
    /**
     * Command sequence for climb step 13
     * @param climb SK22Climb Subsystem
     * @return The command containing the action for step 13
     */
    public static Command getStep13(SK22Climb climb)
    {
        return new MoveComplexArmLiftCommand(climb, ClimbConstants.COMPLEX_FULL_EXTEND);
    }
}
