/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Set;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTools.AutoPaths;
import frc.robot.AutoTools.SK22CommandBuilder;
import frc.robot.AutoTools.TrajectoryBuilder;
import frc.robot.AutoTools.SK22Paths.RunJson;
import frc.robot.subsystems.SK22Drive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the
 * {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure
 * of the robot (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer
{
    private final TrajectoryBuilder    segmentCreator      =
            new TrajectoryBuilder(Constants.SPLINE_DIRECTORY);
    private final SK22CommandBuilder   pathBuilder         =
            new SK22CommandBuilder(Constants.AUTOS_FOLDER_DIRECTORY, segmentCreator);
    private SendableChooser<AutoPaths> autoCommandSelector = new SendableChooser<AutoPaths>();
    private SendableChooser<Command>   driveModeSelector   = new SendableChooser<Command>();

    private SendableChooser<Trajectory> splineCommandSelector = new SendableChooser<Trajectory>();

    // The robot's subsystems are defined here...

    // Drivetrain is the only subsystem that is not optional.
    private final SK22Drive driveSubsystem = new SK22Drive();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        configureShuffleboard();
    }

    private void configureShuffleboard()
    {
        addPossibleAutos();

        SmartDashboard.putData("Auto Chooser", autoCommandSelector);
        SmartDashboard.putData("Drive Mode", driveModeSelector);
        SmartDashboard.putData("JSON Selector", splineCommandSelector);
    }

    /**
     * Reset the encoders and gyro in the drive subsystem. This should be called on boot
     * and when initializing auto and reset modes.
     */
    public void resetDriveSubsystem()
    {
        driveSubsystem.resetEncoders();
        driveSubsystem.resetGyro();
    }

    /**
     * Adds the possible auto commands to the Shuffleboard list depending on the auto
     * segments that are present according to the {@link TrajectoryBuilder}. Adds the auto
     * commands to their respective {@link SendableChooser} to be used by
     * {@link SmartDashboard}.
     */
    public void addPossibleAutos()
    {
        // Adding all JSON paths
        Set<String> splineDirectory = segmentCreator.getTrajectoryNames();
        for (String pathname : splineDirectory)
        {
            splineCommandSelector.addOption(pathname, segmentCreator.getTrajectory(pathname));
        }
        String firstJSON = splineDirectory.stream().findFirst().get();
        splineCommandSelector.setDefaultOption(firstJSON, segmentCreator.getTrajectory(firstJSON));

        autoCommandSelector.addOption("Run Json", new RunJson(splineCommandSelector));

        // Checking dependencies for autos before giving option to run
        // Adds a majority of autos that have multiple segments
        pathBuilder.displayPossibleAutos(autoCommandSelector);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return autoCommandSelector.getSelected().getCommand(segmentCreator,
            driveSubsystem::makeTrajectoryCommand);
    }
}
