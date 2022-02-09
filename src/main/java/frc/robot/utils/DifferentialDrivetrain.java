package frc.robot.utils;

/**
 * An interface used by drive subsystems. It defines what functions the drive subsystem
 * must have in order for a class to make trajectories and trajectory commands with the
 * subsytem.
 */
public interface DifferentialDrivetrain
{
    /**
     * A function that sets the left and right motor speeds using voltage
     * 
     * @param leftSpeeds
     *            The voltage to be sent to the left motor group
     * @param rightSpeeds
     *            The voltage to be sent to the right motor group
     */
    public void tankDriveVolts(double leftSpeeds, double rightSpeeds);
}
