package frc.robot.AutoTools;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Optional;
import java.util.Scanner;
import java.util.Set;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.AutoTools.SK22Paths.DoNothing;
import frc.robot.AutoTools.SK22Paths.Drive1mForwardBackward;
import frc.robot.AutoTools.SK22Paths.DriveSplineCanned;
import frc.robot.AutoTools.SK22Paths.FourBallTerminal1ALHHH;
import frc.robot.AutoTools.SK22Paths.FourBallTerminal2BLHHH;
import frc.robot.AutoTools.SK22Paths.FourBallTerminalRadial1AHHHH;
import frc.robot.AutoTools.SK22Paths.FourBallTerminalRadial2BHHHH;
import frc.robot.AutoTools.SK22Paths.Taxi;
import frc.robot.AutoTools.SK22Paths.ThreeBallTerminal1A;
import frc.robot.AutoTools.SK22Paths.ThreeBallTerminal2A;
import frc.robot.AutoTools.SK22Paths.TwoBallRadialHH;
import frc.robot.commands.DoNothingCommand;
import frc.robot.subsystems.SK22Intake;
import frc.robot.subsystems.SK22Launcher;
import frc.robot.subsystems.SK22Transfer;

/**
 * A class that adds auto path options to smart dashboard if the segments to create the
 * paths are present.
 */
public class SK22CommandBuilder
{
    private File                     pathDirectory;
    private File[]                   files;
    private Map<String, Set<String>> dependencies = new HashMap<String, Set<String>>();
    private TrajectoryBuilder        pathBuilder;

    private AutoPaths defaultAuto;

    private Set<AutoPaths> autoPaths = new HashSet<AutoPaths>();

    /**
     * Creates a new SK22CommandBuilder. It will create all the dependencies for each auto
     * path using the Autos file. It can then be used to display the possible auto paths
     * using the dependencies and the currently created path segments.
     * 
     * @param directory
     *            The file path to get to the Autos directory made by Pathweaver
     * @param pathBuilder
     *            A {@link TrajectoryBuilder} that contains the generated path segments
     * @param intake
     *            The subsystem needed to get cargo
     * @param transfer
     *            The subsystem needed to transfer cargo from the intake to the launcher
     * @param launcher
     *            The subsystem needed to launch the cargo
     */
    public SK22CommandBuilder(String directory, TrajectoryBuilder pathBuilder,
        Optional<SK22Intake> intake, Optional<SK22Transfer> transfer,
        Optional<SK22Launcher> launcher)
    {
        this.pathBuilder = pathBuilder;
        pathDirectory = new File(Filesystem.getDeployDirectory(), directory);
        files = pathDirectory.listFiles();
        for (File file : files)
        {
            try
            {
                Scanner myReader = new Scanner(file);
                Set<String> fileDependency = new HashSet<String>();
                while (myReader.hasNextLine())
                {
                    fileDependency.add(myReader.nextLine());
                }
                dependencies.put(file.getName(), fileDependency);
                myReader.close();
            }
            catch (FileNotFoundException e)
            {
                System.out.println(e);
            }
        }


        defaultAuto = new TwoBallRadialHH(intake, transfer, launcher);
        // Adding all the auto paths to the set
        autoPaths.add(new Taxi());
        // autoPaths.add(new TwoBallRadialHH(intake, transfer, launcher));
        autoPaths.add(new ThreeBallTerminal1A());
        autoPaths.add(new ThreeBallTerminal2A());
        autoPaths.add(new FourBallTerminal1ALHHH());
        autoPaths.add(new FourBallTerminal2BLHHH());
        autoPaths.add(new FourBallTerminalRadial1AHHHH());
        autoPaths.add(new FourBallTerminalRadial2BHHHH());
        autoPaths.add(new Drive1mForwardBackward());

    }

    /**
     * Gets all the possible autos that can be made using the Autos file from Pathweaver
     * and the paths gotten from {@link TrajectoryBuilder}.
     * 
     * @return A set containing all possible paths that can be created according to the
     *         Autos file and the paths that were able to be generated by the
     *         {@link TrajectoryBuilder} class handed in the constructor.
     */
    private Set<String> getPossibleAutos()
    {
        Set<String> commands = new HashSet<String>();
        Set<String> pathNames = new HashSet<String>();
        pathNames.addAll(dependencies.keySet());
        for (String path : pathNames)
        {
            if (pathBuilder.hasTrajectories(dependencies.get(path)))
            {
                commands.add(path);
            }
        }
        return commands;
    }

    /**
     * Adds all the options to the shuffleboard using the information given from the Autos
     * path segments. It will add the option if all the segments for the specified path is
     * present.
     * 
     * @param displayMethod
     *            The function required to add the options for the paths
     */
    public void displayPossibleAutos(SendableChooser<AutoPaths> displayMethod)
    {
        /**
         * A set of autos that are possible with the known auto paths and their
         * dependencies
         */
        Set<String> possibleAutos = getPossibleAutos();

        // Default
        displayMethod.setDefaultOption("Do Nothing", new DoNothing());

        // Test Paths

        displayMethod.addOption("Drive canned path", new DriveSplineCanned());

        if (possibleAutos.contains("Drive1mForwardsBackwards"))
        {
            displayMethod.addOption("Drive forwards then backwards 1m",
                new Drive1mForwardBackward());
        }

        // Adds paths if they are possible to make
        for (AutoPaths auto : autoPaths)
        {
            if (possibleAutos.contains(auto.getPathweaverName()))
            {
                displayMethod.addOption(auto.getName(), auto);
            }

            displayMethod.setDefaultOption(defaultAuto.getName(), defaultAuto);
        }
    }
}
