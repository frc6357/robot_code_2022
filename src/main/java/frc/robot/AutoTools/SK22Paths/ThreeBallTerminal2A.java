package frc.robot.AutoTools.SK22Paths;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.AutoTools.AutoPaths;
import frc.robot.AutoTools.RamseteTrajectoryMethod;
import frc.robot.AutoTools.TrajectoryBuilder;
import frc.robot.commands.DoNothingCommand;
import frc.robot.commands.SetIntakePositionCommand;
import frc.robot.commands.SetLauncherSpeedCommand;
import frc.robot.commands.ShootBallsCommand;
import frc.robot.commands.subcommands.TimeDelayCommand;
import frc.robot.subsystems.SK22Intake;
import frc.robot.subsystems.SK22Launcher;
import frc.robot.subsystems.SK22Transfer;

/**
 * A class that contains the auto that starts at Tarmac 2A, shoots the ball, grabs ball 2
 * and terminal ball, then shoots the two balls
 */
public class ThreeBallTerminal2A implements AutoPaths
{
    private Optional<SK22Intake>   intake;
    private Optional<SK22Transfer> transfer;
    private Optional<SK22Launcher> launcher;

    /**
     * Creates a new ThreeBallTerminal2A command
     * 
     * @param intake
     *            The subsystem needed to get cargo
     * @param transfer
     *            The subsystem needed to transfer cargo from the intake to the launcher
     * @param launcher
     *            The subsystem needed to launch the cargo
     */
    public ThreeBallTerminal2A(Optional<SK22Intake> intake, Optional<SK22Transfer> transfer,
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
            // Leaves the tarmac and shoots the balls
            getParallelCommand(segmentCreator, trajectoryCreator),
            getShootCommand(),

            // Goes to terminal and picks up two balls then goes to shooting position
            new ParallelCommandGroup(
                // Gets ready for final launch speed
                getLauncherCommand(LauncherConstants.MAX_SPEED_PRESET),
                // Drives path to terminal then to shooting position
                // Waits at terminal
                getTerminalCommand(segmentCreator, trajectoryCreator)),
            
            // Shoots the loaded balls
            getShootCommand());
    }

    /**
     * Makes the parallel command group that is required when exiting the tarmac
     * 
     * @param segmentCreator
     *            The class that is used to access the auto segments
     * @param trajectoryCreator
     *            The method used to create the trajectory using Ramsete controller
     * @return
     *            The auto command required to exit the tarmac
     */
    private Command getParallelCommand(TrajectoryBuilder segmentCreator,
        RamseteTrajectoryMethod trajectoryCreator)
    {
        return new ParallelCommandGroup(
            trajectoryCreator
                .createTrajectory(segmentCreator.getTrajectory("Low to Ball 2 (LH)"), true),
            getIntakeCommand(), 
            getLauncherCommand(AutoConstants.AUTO_LAUNCH_SPEED), 
            new TimeDelayCommand(5000));

    }

    /**
     * Gets the intake command required to set the intake outwards and turn on the motors
     * 
     * @return The intake command to extend the intake
     */
    private Command getIntakeCommand()
    {
        return (intake.isPresent() && transfer.isPresent())
            ? new SetIntakePositionCommand(intake.get(), transfer.get(), true)
            : new DoNothingCommand();
    }

    /**
     * Makes and gets the command required to set the launcher to a setpoint
     * 
     * @param launchSpeed
     *            The desired speed of the launcher in RPM
     * @return A command that ends when the launcher reaches the setpoint
     */
    private Command getLauncherCommand(double launchSpeed)
    {
        return (launcher.isPresent())
            ? (new SetLauncherSpeedCommand(launcher.get(), launchSpeed))
            : new DoNothingCommand();
    }

    /**
     * A command that turns on the transfer motors to shoot the ball out of the launcher
     * 
     * @return Turns the transfer motor on for a set amount of time.
     */
    private Command getShootCommand()
    {
        return new ParallelDeadlineGroup(
            new TimeDelayCommand(5000),
            (launcher.isPresent() && transfer.isPresent())
                ? new ShootBallsCommand(launcher.get(), transfer.get()) : new DoNothingCommand());
    }

    /**
     * A command that deals with all of the terminal related commands. It will go to the
     * terminal, wait a set amount of time, then goes to the shooting position, while
     * ensuring that the intake motor is set to the correct speeds.
     * 
     * @param segmentCreator
     *            The class that is used to access the auto segments
     * @param trajectoryCreator
     *            The method used to create the trajectory using Ramsete controller
     * @return The auto command that ends when all the paths are finished
     */
    private Command getTerminalCommand(TrajectoryBuilder segmentCreator,
        RamseteTrajectoryMethod trajectoryCreator)
    {
        return new ParallelCommandGroup(
            // Turns on correct intake motors again
            getIntakeCommand(),
            // Drives to terminal, waits, then drives away from terminal
            new SequentialCommandGroup(
                trajectoryCreator.createTrajectory(segmentCreator.getTrajectory("Ball 2 to Terminal"),
                    false),
                new TimeDelayCommand(2000),
                trajectoryCreator.createTrajectory(segmentCreator.getTrajectory("Terminal to Shoot"),
                    false)));
    }
}
