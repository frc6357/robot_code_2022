package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.subcommands.CurrentChangeCommand;
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

/**
 * Sequence of climb commands
 */
public final class ClimbSequence
{
    private ClimbSequence()
    {
        // Do nothing here
    }

    /**
     * The simple arms would raise to go under the bar while the complex pivot forward so they are out of the way
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
     * Once under the bar the simple arm will lower picking the robot up while the complex would extend past the arm
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
     * The complex arm would pivot downwards to the bar
     * @param climb SK22Climb Subsystem
     * @return The command containing the actions for step 3
     */
    public static Command getStep3(SK22Climb climb)
    {
        return new PivotComplexArmCommand(climb, ClimbConstants.COMPLEX_PARTIAL_STRAIGHTEN);
    }

    /**
     * The complex arms would retract slowly and ratchet until the current threshold is meet
     * @param climb SK22Climb Subsystem
     * @return The command containing the actions for step 4
     */
    public static Command getStep4(SK22Climb climb)
    {
        ParallelCommandGroup step4 = new ParallelCommandGroup();
        step4.addCommands(new MoveComplexArmLiftCommand(climb, ClimbConstants.COMPLEX_FULL_RETRACT));
        step4.addCommands(new TimeDelayCommand(ClimbConstants.STEP4_DELAY_MILLIS));
        step4.addCommands(new CurrentChangeCommand(climb, ClimbConstants.CURRENT_THRESHOLD));
        return step4;
    }

    /**
     * Once the arm hits it would retract quickly and straighten the complex arms. The simple will be put into neutral
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
     * The simple arm would raise to be released from the first bar
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
     * The simple arm would tilt back so it can be released from the first bar
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
     * The simple arm would be lowered for arm clearence
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
     * Pressure would released from the simple arm and the arm would also be straightened
     * @param climb SK22Climb Subsystem
     * @return The command containing the actions for step 9
     */
    public static Command getStep9(SK22Climb climb)
    {
        ParallelCommandGroup step9 = new ParallelCommandGroup();
        //TODO: Check if this would be the correct command to use in this situation
        step9.addCommands(new NeutralRaiseSimpleArmCommand(climb));
        step9.addCommands(new StraightenSimpleArmCommand(climb));
        return step9;
    }

    /**
     * The simple arm would be lowered down so to raise the robot/
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
     * Partaily extend the arm so it has clearence
     * @param climb SK22Climb Subsystem
     * @return The command containing the action for step 11
     */
    public static Command getStep11(SK22Climb climb)
    {
        return new MoveComplexArmLiftCommand(climb, ClimbConstants.COMPLEX_PARTIAL_EXTEND);
    }

    /**
     * Pivot the complex arm backwards so it can go underneath the bar
     * @param climb SK22Climb Subsystem
     * @return The command containing the action for step 12
     */
    public static Command getStep12(SK22Climb climb)
    {
        return new PivotComplexArmCommand(climb, ClimbConstants.COMPLEX_FULL_TILT);
    }
    
    /**
     * Pivot arm forwards so it would be in the same situation as it was before.
     * @param climb SK22Climb Subsystem
     * @return The command containing the action for step 13
     */
    public static Command getStep13(SK22Climb climb)
    {
        return new MoveComplexArmLiftCommand(climb, ClimbConstants.COMPLEX_FULL_EXTEND);
    }
}
