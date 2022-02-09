package frc.robot.AutoTools;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.DifferentialDrivetrain;

/**
 * An interface used to allow {@link RobotContainer} to pass in a method needed to create
 * trajectories into the classes that implement the {@link AutoPaths} interface.
 */
public interface RamseteTrajectoryMethod
{
    /**
     * The function that uses the passed in method and uses it to make a trajectory
     * command. Specifically used to run the {@code MakeTrajectoryCommand} function.
     * 
     * @param trajectory
     *            The desired path
     * @param resetOdometry
     *            Whether the odometry should be reset at the beginning of the command or
     *            not
     * @param driveSubsystem
     *            The required subsystem to run the path
     * @return The command that makes the subystem run the path
     */
    Command createTrajectory(Trajectory trajectory, boolean resetOdometry,
        DifferentialDrivetrain driveSubsystem);
}
