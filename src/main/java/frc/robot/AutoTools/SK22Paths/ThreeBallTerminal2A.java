package frc.robot.AutoTools.SK22Paths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTools.AutoPaths;
import frc.robot.AutoTools.RamseteTrajectoryMethod;
import frc.robot.AutoTools.TrajectoryBuilder;
import frc.robot.commands.DoNothingCommand;

/**
 * A class that contains the auto that starts at Tarmac 2A, shoots the ball, grabs ball 2
 * and terminal ball, then shoots the two balls
 */
public class ThreeBallTerminal2A implements AutoPaths
{
    /**
     * A function that gets the name of the auto path as defined by the Pathweaver tool
     * 
     * @return The name of the command in Pathweaver
     */
    public String getPathweaverName()
    {
        return "3T(LHH) Tarmac 2A";
    }
    
    /**
     * A function that gets the name of the auto command
     * 
     * @return The name of the command
     */
    public String getName()
    {
        return "3 Ball Terminal Tarmac 2A";
    }

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
        RamseteTrajectoryMethod trajectoryCreator)
    {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(new DoNothingCommand(), new DoNothingCommand(),
                trajectoryCreator
                    .createTrajectory(segmentCreator.getTrajectory("Low to Ball 2 (LH)"), true)),
            trajectoryCreator.createTrajectory(segmentCreator.getTrajectory("Ball 2 to Terminal"),
                false),
            trajectoryCreator.createTrajectory(segmentCreator.getTrajectory("Terminal to Shoot"),
                false),
            new DoNothingCommand());
    }
}
