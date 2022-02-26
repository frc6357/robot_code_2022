package frc.robot.AutoTools.SK22Paths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTools.AutoPaths;
import frc.robot.AutoTools.RamseteTrajectoryMethod;
import frc.robot.AutoTools.TrajectoryBuilder;

/**
 * A class that contains the path and path name for moving the robot 1m forwards and then
 * immediately 1m backward.
 */
public class Drive1mForwardBackward implements AutoPaths
{
    /**
     * A function that gets the name of the auto path as defined by the Pathweaver tool
     * 
     * @return null, as this class does not have a Pathweaver path
     */
    public String getPathweaverName()
    {
        return "1mForwardsBackwards";
    }

    /**
     * A function that gets the name of the auto command
     * 
     * @return The name of the command
     */
    public String getName()
    {
        return "Drive Forwards Then Backwards 1m";
    }

    /**
     * Creates and returns an autonomous command
     * @param segmentCreator
     *            The class that is used to access the auto segments
     * @param trajectoryCreator
     *            The method used to create the trajectory using Ramsete controller
     * @return The auto command
     */
    public Command getCommand(
        TrajectoryBuilder segmentCreator, RamseteTrajectoryMethod trajectoryCreator)
    {
        return new SequentialCommandGroup(
            trajectoryCreator.createTrajectory(segmentCreator.getTrajectory("1m Forwards"), true),
            trajectoryCreator.createTrajectory(segmentCreator.getTrajectory("1m Backwards"), false));
    }
}
