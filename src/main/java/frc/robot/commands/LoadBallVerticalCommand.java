package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.SK22Transfer;

/**
 * Command that loads ball into the vertical chamber in the case of emergency
 */
public class LoadBallVerticalCommand extends CommandBase
{

    /**
     * Transfer subsystem for the LoadBallVertical Command
     */
    private final SK22Transfer transfer;

    /**
     * A manual override command that allows the operator to transfer balls from
     * the horizontal to the vertical portion of the transfer subsystem.
     * 
     * @param transfer The transfer subsystem on which the command operates.
     */
    public LoadBallVerticalCommand(SK22Transfer transfer)
    {
        this.transfer = transfer;

        addRequirements(transfer);
    }

    /**
     * Method that sets the motor speeds to the ejection speed if the user has set the on
     * variable to true and defaults to zero if on is false
     */
    @Override
    public void initialize()
    {
        // Sets the vertical transfer motor to ejectionspeed
        transfer.setVerticalTransferMotor(TransferConstants.VERTICAL_MOTOR_SPEED);
        // Sets the intake transfer motor to ejectionspeed
        transfer.setIntakeTransferMotor(TransferConstants.INTAKE_MOTOR_SPEED);
        // Sets the ejection motor to intake the ball into the vertical portion
        transfer.setExitTransferMotor(TransferConstants.LOAD_BALL_VERTICAL_SPEED);
    }

    /** {@inheritDoc} */
    @Override
    public void end(boolean interrupted)
    {
        // Turns off the vertical transfer motor
        transfer.setVerticalTransferMotor(0.0);
        // Turns off the intake transfer motor
        transfer.setIntakeTransferMotor(0.0);
        // Turns of the exit transfer motor
        transfer.setExitTransferMotor(0.0);
    }

    /** {@inheritDoc} */
    @Override
    public boolean isFinished()
    {
        return false;
    }

}
