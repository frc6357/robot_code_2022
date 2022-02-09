package frc.robot.AutoTools.SK22Paths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTools.AutoPaths;
import frc.robot.AutoTools.RamseteTrajectoryMethod;
import frc.robot.AutoTools.TrajectoryBuilder;
import frc.robot.commands.DoNothingCommand;
import frc.robot.utils.DifferentialDrivetrain;

/**
 * A class that contains a 4 ball auto that starts on the outside of the tarmac 1A (when
 * speaking radially) and picks up three balls on the way. An extra ball can be picked up
 * from the human player. All of the balls are shot into the high goal
 */
public class T4_HHHH_R1A implements AutoPaths
{
    /**
     * A function that gets the name of the auto command
     * 
     * @return The name of the command
     */
    public String getName()
    {
        return "4 Ball Terminal Tarmac 1A Radial";
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
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                trajectoryCreator.createTrajectory(
                    segmentCreator.getTrajectory("Grab Ball Radial (HH)"), true, driveSubsystem),
                new DoNothingCommand(),     // Intake Set Up
                new DoNothingCommand()),    // Launcher Set Up
            new DoNothingCommand(),         // Launcher Shoot HH
            trajectoryCreator.createTrajectory(segmentCreator.getTrajectory("Ball 1 to Ball 2"),
                true, driveSubsystem),
            trajectoryCreator.createTrajectory(segmentCreator.getTrajectory("Ball 2 to Terminal"),
                false, driveSubsystem),
            trajectoryCreator.createTrajectory(segmentCreator.getTrajectory("Terminal to Shoot"),
                false, driveSubsystem),
            new DoNothingCommand());        // Launcher Shoot HH
    }
}
