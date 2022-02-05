package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SK22Intake;

/**
 * Toggles motor and position of the Intake Subsystem
 */
public class SetIntakePositionCommand extends CommandBase
{
    private final SK22Intake intakeSubsystem;
    private final boolean onCommand;

    /**
     * Creates new SetIntakePositionCommand 
     * @param intakeSubsystem
     *          Ihe Intake Subsystem required for this command.
     *          Used to actuate Intake on a linear axis and set motor speed.
     * @param extended
     *          Whether or not the intake is extended.
     */
    public SetIntakePositionCommand(SK22Intake intakeSubsystem, boolean extended)
    {
        this.intakeSubsystem = intakeSubsystem;
        this.onCommand = extended;

        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize()
    {
        if (onCommand)
        {
            intakeSubsystem.extendIntake();
            intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_MOTOR_SPEED);
        }
        else
        {
            intakeSubsystem.retractIntake();
            intakeSubsystem.setIntakeSpeed(0.0);
        }
    }
    @Override
    public boolean isFinished()
    {
        return true;
    }    
}
