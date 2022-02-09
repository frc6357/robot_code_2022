package frc.robot.AutoTools.SK22Paths;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTools.AutoPaths;
import frc.robot.AutoTools.RamseteTrajectoryMethod;
import frc.robot.AutoTools.TrajectoryBuilder;
import frc.robot.commands.DoNothingCommand;
import frc.robot.utils.DifferentialDrivetrain;

/**
 * An auto path that does absolutely nothing and ends immediately
 */
public class DoNothing implements AutoPaths
{
    /**
     * A function that gets the name of the auto command
     * 
     * @return The name of the command
     */
    public String getName()
    {
        return "Do Nothing";
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
        return new DoNothingCommand();
    }
}
