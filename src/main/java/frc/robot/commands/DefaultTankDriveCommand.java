package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Ports;
import frc.robot.subsystems.SK22Drive;
import frc.robot.utils.FilteredJoystick;
import frc.robot.utils.filters.SlewRateFilter;

/**
 * The default command used when the robot is being controlled via tank drive.
 */
public class DefaultTankDriveCommand extends CommandBase
{
    /**
     * The Drive subsystem for this DefaultDriveCommand.
     */
    private final SK22Drive driveSubsystem;

    /**
     * The Joystick for the Driver.
     */
    private final FilteredJoystick leftJoystickDriver;
    private final FilteredJoystick rightJoystickDriver;

    /**
     * Creates a new DefaultDriveCommand that sets up the member subsystem.
     *
     * @param driveSubsystem The subsystem used by the command to set drivetrain motor speeds
     * @param leftJoystickDriver The joystick controlling the left motors
     * @param rightJoystickDriver The joystick controlling the right motors
     */
    public DefaultTankDriveCommand(SK22Drive driveSubsystem, FilteredJoystick leftJoystickDriver,
        FilteredJoystick rightJoystickDriver)
    {
        this.driveSubsystem = driveSubsystem;
        this.leftJoystickDriver = leftJoystickDriver;
        this.rightJoystickDriver = rightJoystickDriver;

        // Applies a Cubic filter with a Deadband to the left speed axis of the joystick.
        // This Cubic filter will have a moderate curvature with a coefficient of 0.6.
        // The Deadband will have a width of 0.05.
        this.leftJoystickDriver.setFilter(Ports.OI_DRIVER_SPEED_AXIS,
            new SlewRateFilter(DriveConstants.SLEW_FILTER_RATE, true));
        // no deadband here as the SK21Drive implements the deadband 

        // Applies a Cubic Filter with a Deadband to the right speed axis of the joystick.
        // This Cubic filter will have maxmimum curvature with a coefficient of 1.
        // The Deadband will have a width of 0.05.
        this.rightJoystickDriver.setFilter(Ports.OI_DRIVER_SPEED_AXIS,
            new SlewRateFilter(DriveConstants.SLEW_FILTER_RATE, true));

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
        double leftSpeed = leftJoystickDriver.getFilteredAxis(Ports.OI_DRIVER_SPEED_AXIS);
        double rightSpeed = rightJoystickDriver.getFilteredAxis(Ports.OI_DRIVER_SPEED_AXIS);

        driveSubsystem.tankDrive(leftSpeed, rightSpeed);
    }

    // False as default commands are intended to not end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
