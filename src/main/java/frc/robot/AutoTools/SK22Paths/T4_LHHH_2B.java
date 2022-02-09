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
 * A class that contains the four ball auto that starts near the hub of Tarmac 2B then
 * picks up three more balls off the field. A ball can also be picked up off an alliance
 * robot and from the human player at the terminal. The first ball is launched into the
 * low goal, while the others are launched into the high goal.
 */
public class T4_LHHH_2B implements AutoPaths
{
    /**
     * A function that gets the name of the auto command
     * 
     * @return The name of the command
     */
    public String getName()
    {
        return "4 Ball Terminal Tarmac 2B";
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
        return new SequentialCommandGroup(new ParallelCommandGroup(new DoNothingCommand(),     // Launcher Shoot L
            new DoNothingCommand()),        // Set Up Intake
            trajectoryCreator.createTrajectory(
                segmentCreator.getTrajectory("Low to Ball 3 (LH)"), true, driveSubsystem),
            new DoNothingCommand(),         // Launcher Shoot H
            trajectoryCreator.createTrajectory(segmentCreator.getTrajectory("Ball 3 to Ball 2"),
                false, driveSubsystem),
            trajectoryCreator.createTrajectory(segmentCreator.getTrajectory("Ball 2 to Terminal"),
                false, driveSubsystem),
            trajectoryCreator.createTrajectory(segmentCreator.getTrajectory("Terminal to Shoot"),
                false, driveSubsystem),
            new DoNothingCommand());
    }
}
