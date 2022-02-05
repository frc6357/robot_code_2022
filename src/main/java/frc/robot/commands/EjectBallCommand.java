package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.SK22Transfer;

/**
 * A command that ejects balls through the horizontal exit in case of an emergency
 */

public class EjectBallCommand extends CommandBase{

    /**
     * Transfer Subsystem for the ejectball command
     */
    private final SK22Transfer transferSubsystem;

    /**
     * Varible that checks whether the player is turning on the robot or not
     */
    private boolean exitToggle;
 

    public EjectBallCommand(SK22Transfer transferSubsystem, boolean exitToggle)
    {
        this.transferSubsystem = transferSubsystem;
        this.exitToggle = exitToggle;

        addRequirements(transferSubsystem);
    }

    /**
     * Method that sets the motor speeds to the ejection speed if the user has set the on variable to true and defaults
     * to zero if on is false
     */
    @Override
    public void initialize()
    {
        //Sets the exist transfer motor to ejectionspeed or zero based on user input
        this.transferSubsystem.setExitTransferMotor(exitToggle? TransferConstants.BALL_EJECTION_SPEED : 0);
        //Sets the intake transfer motor to ejectionspeed or zero based on user input
        this.transferSubsystem.setIntakeTransferMotor(exitToggle? TransferConstants.BALL_EJECTION_SPEED: 0);
    }

    /**
     * Allows the function to finish
     */
    @Override
    public boolean isFinished()
    {
        return true;
    }
    
}
