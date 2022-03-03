package frc.robot.AutoTools.SK22Paths;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTools.AutoPaths;
import frc.robot.AutoTools.RamseteTrajectoryMethod;
import frc.robot.AutoTools.TrajectoryBuilder;
import frc.robot.commands.DoNothingCommand;
import frc.robot.commands.SetIntakePositionCommand;
import frc.robot.commands.ShootBallsCommand;
import frc.robot.subsystems.SK22Intake;
import frc.robot.subsystems.SK22Launcher;
import frc.robot.subsystems.SK22Transfer;

/**
 * A class that contains the auto path that starts at the outer edge of the tarmac (when
 * speaking in radial terms) and then grabs the ball immediately behind it, then shoots
 * both balls into the high goal.
 */
public class TwoBallRadialHH implements AutoPaths
{
    private Optional<SK22Intake>   intake;
    private Optional<SK22Transfer> transfer;
    private Optional<SK22Launcher> launcher;

    /**
     * Creates a new TwoBallRadialHH command
     * 
     * @param intake
     *            The subsystem needed to get cargo
     * @param transfer
     *            The subsystem needed to transfer cargo from the intake to the launcher
     * @param launcher
     *            The subsystem needed to launch the cargo
     */
    public TwoBallRadialHH(Optional<SK22Intake> intake, Optional<SK22Transfer> transfer,
        Optional<SK22Launcher> launcher)
    {
        this.intake = intake;
        this.transfer = transfer;
        this.launcher = launcher;
    }

    /**
     * A function that gets the name of the auto path as defined by the Pathweaver tool
     * 
     * @return The name of the command in Pathweaver
     */
    public String getPathweaverName()
    {
        return "Radial (HH)";
    }

    /**
     * A function that gets the name of the auto command
     * 
     * @return The name of the command
     */
    public String getName()
    {
        return "2 Ball Radial HH";
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
            new ParallelCommandGroup(
                trajectoryCreator
                    .createTrajectory(segmentCreator.getTrajectory("Grab Ball Radial (HH)"), true),
                intake.isPresent() ?
                    new SetIntakePositionCommand(intake.get(), true) : new DoNothingCommand()),
            (launcher.isPresent() && transfer.isPresent()) ?
                new ShootBallsCommand(launcher.get(), transfer.get()) : new DoNothingCommand());
    }
}
