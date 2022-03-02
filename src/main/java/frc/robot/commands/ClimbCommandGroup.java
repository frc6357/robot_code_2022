package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SK22ComplexClimb;
import frc.robot.subsystems.SK22SimpleClimb;
/**
 * ClimbCommandGroups runs a sequence of commands
 */
public class ClimbCommandGroup extends CommandBase
{

    private final SK22ComplexClimb complexClimb;
    private final SK22SimpleClimb simpleClimb;
    private final ClimbSequence climbSequence;

    /**
     * 
     * @param complexClimb subsytem of the complex climb
     * @param simpleClimb subsytem of the simple climb
     * @param climbSequence A sequnce of method of parallel and sequential commands that run the climb steps
     */
    public ClimbCommandGroup(SK22ComplexClimb complexClimb, SK22SimpleClimb simpleClimb, ClimbSequence climbSequence)
    {
        this.complexClimb = complexClimb;
        this.simpleClimb = simpleClimb;
        this.climbSequence = climbSequence;
    }
    

    @Override
    public void initialize()
    {
        climbSequence.getStep3(complexClimb);
        climbSequence.getStep4(complexClimb);
        climbSequence.getStep5(complexClimb, simpleClimb);
        climbSequence.getStep6(complexClimb, simpleClimb);
        climbSequence.getStep7(simpleClimb);
        climbSequence.getStep8(simpleClimb);
        climbSequence.getStep9(simpleClimb);
        climbSequence.getStep10(simpleClimb);
        climbSequence.getStep11(complexClimb);
        climbSequence.getStep12(complexClimb);
        climbSequence.getStep13(complexClimb);
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
