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
    private final SK22Transfer transferSubsystem;
    /**
     * Varible that checks whether the player is turning on the robot or not
     */
    private boolean verticalToggle;

    /**
     * A manual override command that allows the operator to transfer balls from
     * the horizontal to the vertical portion of the transfer subsystem.
     * 
     * @param transferSubsystem The transfer subsystem on which the command operates.
     * @param verticalToggle If true, the motors are started to feed balls into the
     * vertical transfer area. If false, the motors are stopped.
     */
    public LoadBallVerticalCommand(SK22Transfer transferSubsystem, boolean verticalToggle)
    {
        this.transferSubsystem = transferSubsystem;
        this.verticalToggle = verticalToggle;

        addRequirements(transferSubsystem);
    }

    /**
     * Method that sets the motor speeds to the ejection speed if the user has set the on
     * variable to true and defaults to zero if on is false
     */
    @Override
    public void initialize()
    {
        //Sets the vertical transfer motor to ejectionspeed or zero based on user input
        this.transferSubsystem
            .setVerticalTransferMotor(verticalToggle ? TransferConstants.BALL_EJECTION_SPEED : 0);
        //Sets the intake transfer motor to ejectionspeed or zero based on user input
        this.transferSubsystem.setIntakeTransferMotor(
            verticalToggle ? TransferConstants.TRANSFER_BALL_VERTICAL_SHAFT_SPEED : 0);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }

}
