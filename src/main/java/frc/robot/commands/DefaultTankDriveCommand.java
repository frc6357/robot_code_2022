package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ports;
import frc.robot.subsystems.SK21Drive;
import frc.robot.utils.FilteredJoystick;
import frc.robot.utils.filters.CubicDeadbandFilter;

public class DefaultTankDriveCommand extends CommandBase{
     /**
     * The Drive subsystem for this DefaultDriveCommand.
     */
    private final SK21Drive driveSubsystem;

    /**
     * The Joystick for the Driver.
     */
    private final FilteredJoystick leftJoystickDriver;
    private final FilteredJoystick rightJoystickDriver;

    /**
     * Creates a new DefaultDriveCommand that sets up the member subsystem.
     *
     * @param driveSubsystem
     *            The subsystem used by the command to set drivetrain motor speeds
     * @param joystickDriver
     *            The Joystick used for driving
     */
    public DefaultTankDriveCommand(SK21Drive driveSubsystem, 
                    FilteredJoystick leftJoystickDriver, FilteredJoystick rightJoystickDriver)
    {
        this.driveSubsystem = driveSubsystem;
        this.leftJoystickDriver = leftJoystickDriver;
        this.rightJoystickDriver = rightJoystickDriver;
        
        // Applies a Cubic filter with a Deadband to the left speed axis of the joystick.
        // This Cubic filter will have a moderate curvature with a coefficient of 0.6.
        // The Deadband will have a width of 0.05.
        leftJoystickDriver.setFilter(Ports.OIDriverSpeedAxis, new CubicDeadbandFilter(0.0, 0.0,0.8, true));
        // no deadband here as the SK21Drive implements the deadband 
        

        // Applies a Cubic Filter with a Deadband to the right speed axis of the joystick.
        // This Cubic filter will have maxmimum curvature with a coefficient of 1.
        // The Deadband will have a width of 0.05.
        rightJoystickDriver.setFilter(Ports.OIDriverSpeedAxis, new CubicDeadbandFilter(0.0, 0.0, 0.8 ,true));

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    /**
     * This method, which is usually run every 20ms, takes in the filtered joystick values
     * and sets the speeds that the drivetrain motors need to achieve.
     */
    @Override
    public void execute()
    {
        /*
         * TODO: This used to call the slowMode() method on the driveSubsystem dependent
         * upon the value of the joystick bumper. This is now handled by creating a
         * joystick button command which calls setMaxOutput(). Check to make sure this
         * still works!
         */
        double leftSpeed = leftJoystickDriver.getFilteredAxis(Ports.OIDriverSpeedAxis);
        double rightSpeed = rightJoystickDriver.getFilteredAxis(Ports.OIDriverSpeedAxis);

        driveSubsystem.tankDrive(leftSpeed, rightSpeed);
    }

    // False as default commands are intended to not end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
