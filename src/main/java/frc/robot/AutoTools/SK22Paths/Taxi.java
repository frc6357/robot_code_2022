package frc.robot.AutoTools.SK22Paths;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTools.AutoPaths;
import frc.robot.AutoTools.RamseteTrajectoryMethod;
import frc.robot.AutoTools.TrajectoryBuilder;
import frc.robot.utils.DifferentialDrivetrain;

/**
 * The class that contains the auto path that simply taxis the robot off of the tarmac
 */
public class Taxi implements AutoPaths
{
    /**
     * A function that gets the name of the auto command
     * 
     * @return The name of the command
     */
    public String getName()
    {
        return "Taxi";
    }

    /**
     * Creates and returns an autonomous command
     * 
     * @param driveSubsystem
     *            The subsystem required to run the autonomous command
     * @param segmentCreator
     *            The class that is used to access the auto segments
     * @param trajectoryCreator
     *            The method used to create the trajectory using Ramsete controller
     * @return The auto command
     */
    public Command getCommand(DifferentialDrivetrain driveSubsystem,
        TrajectoryBuilder segmentCreator, RamseteTrajectoryMethod trajectoryCreator)
    {
        return trajectoryCreator.createTrajectory(segmentCreator.getTrajectory("Simple Taxi"),
            true, driveSubsystem);
    }
}
