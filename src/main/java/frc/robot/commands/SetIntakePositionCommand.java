package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.SK22Intake;
import frc.robot.subsystems.SK22Transfer;

/**
 * Toggles motor and position of the Intake Subsystem
 */
public class SetIntakePositionCommand extends CommandBase
{
    private final SK22Intake intake;
    private final SK22Transfer transfer;
    private final boolean    onCommand;

    /**
     * Creates new SetIntakePositionCommand
     * 
     * @param intake
     *            Ihe Intake Subsystem required for this command. Used to actuate Intake
     *            on a linear axis and set motor speed.
     * @param transfer
     *            The transfer subsystem needed to intake the ball
     * @param extended
     *            Whether or not the intake is extended.
     */
    public SetIntakePositionCommand(SK22Intake intake, SK22Transfer transfer,
        boolean extended)
    {
        this.intake = intake;
        this.transfer = transfer;
        this.onCommand = extended;

        addRequirements(intake, transfer);
    }

    @Override
    public void initialize()
    {
        if (onCommand)
        {
            intake.setIntakeSpeed(IntakeConstants.INTAKE_MOTOR_SPEED);
            transfer.setIntakeTransferMotorSpeed(TransferConstants.INTAKE_MOTOR_SPEED);
        }
        else
        {
            intake.setIntakeSpeed(0.0);
            transfer.setIntakeTransferMotorSpeed(0.0);
        }
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
