package frc.robot.AutoTools;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Interface that states the functions that any auto path class must contain.
 */
public interface AutoPaths
{
    /**
     * A function that gets the name of the auto path as defined by the Pathweaver tool
     * 
     * @return The name of the command in Pathweaver
     */
    public String getPathweaverName();

    /**
     * A function that gets the name of the auto command
     * 
     * @return The name of the command
     */
    public String getName();

    /**
     * Creates and returns an autonomous command
     * 
     * @param segmentCreator
     *            The class that is used to access the auto segments
     * @param trajectoryCreator
     *            The method used to create the trajectory using Ramsete controller
     * @return The auto command
     */
    public Command getCommand(TrajectoryBuilder segmentCreator,
        RamseteTrajectoryMethod trajectoryCreator);
}
