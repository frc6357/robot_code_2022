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
import frc.robot.subsystems.SK22SimpleClimb;
import frc.robot.subsystems.SK22ComplexClimb;

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
     * @param complexClimb SK22ComplexClimb Subsystem
     * @param simpleClimb SK22SimpleClimb Subsystem
     * @return The command containing the actions for step 1 
     */
    public static Command getStep1(SK22ComplexClimb complexClimb, SK22SimpleClimb simpleClimb)
    {
        ParallelCommandGroup step1 = new ParallelCommandGroup();
        step1.addCommands(new RaiseSimpleArmCommand(simpleClimb));
        step1.addCommands(new PivotComplexArmCommand(complexClimb, ClimbConstants.COMPLEX_FULL_TILT));
        return step1;
    }

    /**
     * Once under the bar the simple arm will lower picking the robot up while the complex would extend past the arm
     * @param complexClimb SK22ComplexClimb Subsystem
     * @param simpleClimb SK22SimpleClimb Subsystem
     * @return The command containing the actions for step 2
     */
    public static Command getStep2(SK22ComplexClimb complexClimb, SK22SimpleClimb simpleClimb)
    {
        ParallelCommandGroup step2 = new ParallelCommandGroup();
        step2.addCommands(new LowerSimpleArmCommand(simpleClimb));
        step2.addCommands(new MoveComplexArmLiftCommand(complexClimb, ClimbConstants.COMPLEX_FULL_EXTEND));
        return step2;
    }

    /**
     * The complex arm would pivot downwards to the bar
     * @param complexClimb SK22ComplexClimb Subsystem
     * @return The command containing the actions for step 3
     */
    public static Command getStep3(SK22ComplexClimb complexClimb)
    {
        return new PivotComplexArmCommand(complexClimb, ClimbConstants.COMPLEX_PARTIAL_STRAIGHTEN);
    }

    /**
     * The complex arms would retract slowly and ratchet until the current threshold is meet
     * @param complexClimb SK22ComplexClimb Subsystem
     * @return The command containing the actions for step 4
     */
    public static Command getStep4(SK22ComplexClimb complexClimb)
    {
        ParallelCommandGroup step4 = new ParallelCommandGroup();
        step4.addCommands(new MoveComplexArmLiftCommand(complexClimb, ClimbConstants.COMPLEX_FULL_RETRACT));
        step4.addCommands(new TimeDelayCommand(ClimbConstants.STEP4_DELAY_MILLIS));
        step4.addCommands(new CurrentChangeCommand(complexClimb, ClimbConstants.CURRENT_THRESHOLD));
        return step4;
    }

    /**
     * Once the arm hits it would retract quickly and straighten the complex arms. The simple will be put into neutral
     * @param complexClimb SK22ComplexClimb Subsystem
     * @param simpleClimb SK22SimpleClimb Subystem
     * @return The command containing the actions for step 5
     */
    public static Command getStep5(SK22ComplexClimb complexClimb, SK22SimpleClimb simpleClimb)
    {
        ParallelCommandGroup step5 = new ParallelCommandGroup();
        step5.addCommands(new NeutralTiltSimpleArmCommand(simpleClimb));
        step5.addCommands(new NeutralRaiseSimpleArmCommand(simpleClimb));
        step5
            .addCommands(new MoveComplexArmLiftCommand(complexClimb, ClimbConstants.COMPLEX_FULL_RETRACT));
        step5
            .addCommands(new PivotComplexArmCommand(complexClimb, ClimbConstants.COMPLEX_FULL_STRAIGHTEN));
        return step5;
    }

    /**
     * The simple arm would raise to be released from the first bar
     * @param complexClimb SK22ComplexClimb Subsystem
     * @param simpleClimb SK22SimpleClimb Subsystem
     * @return The command containing the actions for step 6
     */
    public static Command getStep6(SK22ComplexClimb complexClimb, SK22SimpleClimb simpleClimb)
    {
        ParallelCommandGroup step6 = new ParallelCommandGroup();
        step6.addCommands(new RaiseSimpleArmCommand(simpleClimb));
        step6.addCommands(new TimeDelayCommand(ClimbConstants.STEP6_DELAY_MILLIS));
        return step6;
    }

    /**
     * The simple arm would tilt back so it can be released from the first bar
     * @param simpleClimb SK22simpleClimb Subsystem
     * @return The command containing the actions for step 7
     */
    public static Command getStep7(SK22SimpleClimb simpleClimb)
    {
        ParallelCommandGroup step7 = new ParallelCommandGroup();
        step7.addCommands(new TiltSimpleArmCommand(simpleClimb));
        step7.addCommands(new TimeDelayCommand(ClimbConstants.STEP7_DELAY_MILLIS));
        return step7;
    }

    /**
     * The simple arm would be lowered for arm clearence
     * @param simpleClimb SK22Climb Subsystem
     * @return The command containing the actions for step 8
     */
    public static Command getStep8(SK22SimpleClimb simpleClimb)
    {
        ParallelCommandGroup step8 = new ParallelCommandGroup();
        step8.addCommands(new LowerSimpleArmCommand(simpleClimb));
        step8.addCommands(new TimeDelayCommand(ClimbConstants.STEP8_DELAY_MILLIS));
        return step8;
    }

    /**
     * Pressure would released from the simple arm and the arm would also be straightened
     * @param simpleClimb SK22Climb Subsystem
     * @return The command containing the actions for step 9
     */
    public static Command getStep9(SK22SimpleClimb simpleClimb)
    {
        ParallelCommandGroup step9 = new ParallelCommandGroup();
        //TODO: Check if this would be the correct command to use in this situation
        step9.addCommands(new NeutralRaiseSimpleArmCommand(simpleClimb));
        step9.addCommands(new StraightenSimpleArmCommand(simpleClimb));
        return step9;
    }

    /**
     * The simple arm would be lowered down so to raise the robot/
     * @param simpleClimb SK22Climb Subsystem
     * @return The command containing the actions for step 10
     */
    public static Command getStep10(SK22SimpleClimb simpleClimb)
    {
        ParallelCommandGroup step10 = new ParallelCommandGroup();
        step10.addCommands(new LowerSimpleArmCommand(simpleClimb));
        step10.addCommands(new TimeDelayCommand(ClimbConstants.STEP10_DELAY_MILLIS));
        return step10;
    }

    /**
     * Partaily extend the arm so it has clearence
     * @param complexClimb SK22Climb Subsystem
     * @return The command containing the action for step 11
     */
    public static Command getStep11(SK22ComplexClimb complexClimb)
    {
        return new MoveComplexArmLiftCommand(complexClimb, ClimbConstants.COMPLEX_PARTIAL_EXTEND);
    }

    /**
     * Pivot the complex arm backwards so it can go underneath the bar
     * @param complexClimb SK22Climb Subsystem
     * @return The command containing the action for step 12
     */
    public static Command getStep12(SK22ComplexClimb complexClimb)
    {
        return new PivotComplexArmCommand(complexClimb, ClimbConstants.COMPLEX_FULL_TILT);
    }
    
    /**
     * Pivot arm forwards so it would be in the same situation as it was before.
     * @param complexClimb SK22ComplexClimb Subsystem
     * @return The command containing the action for step 13
     */
    public static Command getStep13(SK22ComplexClimb complexClimb)
    {
        return new MoveComplexArmLiftCommand(complexClimb, ClimbConstants.COMPLEX_FULL_EXTEND);
    }
}
