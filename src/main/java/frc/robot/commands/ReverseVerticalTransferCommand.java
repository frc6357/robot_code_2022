package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.SK22Launcher;
import frc.robot.subsystems.SK22Transfer;

/**
 * A command allowing the vertical ball transfer motor to be run backwards.
 */
public class ReverseVerticalTransferCommand extends CommandBase
{
    /**
     * Transfer subsystem for the LoadBallVertical Command
     */
    private final SK22Launcher launcher;

    /**
     * A manual override command that allows the operator to reverse the
     * vertical transfer belt if a ball gets jammed
     * 
     * @param transfer The transfer subsystem on which the command operates.
     */
    public ReverseVerticalTransferCommand(SK22Launcher launcher)
    {
        this.launcher = launcher; 

        launcher.setVerticalTransferMotor(0.0);

        addRequirements(launcher);
    }

    /**
     * Method that sets the motor speeds to the ejection speed if the user has set the on
     * variable to true and defaults to zero if on is false
     */
    @Override
    public void initialize()
    {
        // Sets the vertical transfer motor speed
        launcher.setVerticalTransferMotor(-TransferConstants.VERTICAL_MOTOR_SPEED);
    }

    /** {@inheritDoc} */
    @Override
    public void end(boolean bInterrupted)
    {
        // Turns off the vertical transfer motor
        launcher.setVerticalTransferMotor(0.0);
    }

    /** {@inheritDoc} */
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
