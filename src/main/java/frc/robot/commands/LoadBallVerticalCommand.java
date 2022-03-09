package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.SK22Launcher;
import frc.robot.subsystems.SK22Transfer;

/**
 * Command that loads ball into the base of vertical shaft in the case of emergency
 */
public class LoadBallVerticalCommand extends CommandBase
{

    /**
     * Transfer subsystem for the LoadBallVertical Command
     */
    private final SK22Transfer transfer;

    private final SK22Launcher launcher;

    /**
     * A manual override command that allows the operator to transfer balls from
     * the intake to the base of the vertical shaft of the transfer subsystem.
     * 
     * @param transfer The transfer subsystem on which the command operates.
     */
    public LoadBallVerticalCommand(SK22Transfer transfer, SK22Launcher launcher)
    {
        this.transfer = transfer;
        this.launcher = launcher;

        addRequirements(transfer);
    }

    /**
     * Method that sets the motor speeds to the ejection speed if the user has set the on
     * variable to true and defaults to zero if on is false
     */
    @Override
    public void initialize()
    {
        // Turns of the exit transfer motor
        launcher.setExitTransferMotor(0.0);
        // Sets the intake transfer motor to ejectionspeed
        transfer.setIntakeTransferMotor(TransferConstants.INTAKE_MOTOR_SPEED);
    }

    /** {@inheritDoc} */
    @Override
    public void end(boolean interrupted)
    {
        // Turns off the intake transfer motor
        transfer.setIntakeTransferMotor(0.0);
    }

    /** {@inheritDoc} */
    @Override
    public boolean isFinished()
    {
        return false;
    }

}
