package frc.robot.AutoTools.SK22Paths;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoTools.AutoPaths;
import frc.robot.AutoTools.RamseteTrajectoryMethod;
import frc.robot.AutoTools.TrajectoryBuilder;

/**
 * An auto path that runs the json that is selected by the chooser
 */
public class RunJson implements AutoPaths
{
    private SendableChooser<Trajectory> jsonChooser;

    /**
     * Creates a new RunJson class that will used to make an auto path with the selected
     * json.
     * 
     * @param jsonChooser
     *            A sendable chooser that will decide what json is selected.
     */
    public RunJson(SendableChooser<Trajectory> jsonChooser)
    {
        this.jsonChooser = jsonChooser;
    }

    /**
     * A function that gets the name of the auto command
     * 
     * @return The name of the command
     */
    public String getName()
    {
        return "Run Json";
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
        return makeTrajectoryFromJSON(trajectoryCreator);
    }

    private Command makeTrajectoryFromJSON(RamseteTrajectoryMethod trajectoryCreator)
    {
        return trajectoryCreator.createTrajectory(jsonChooser.getSelected(), true);
    }
}
