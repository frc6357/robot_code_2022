// A class used to pre-build trajectories from JSON files placed in the 
// deploy/paths directory of the RoboRio. This is invoked from robot_init
// to ensure that all the time-consuming trajectory creation is done well
// before autonomous mode starts.

package frc.robot.utils;

import java.io.File;
import java.io.IOException;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the
 * {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure
 * of the robot (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class TrajectoryBuilder
{

    private Map<String, Trajectory> m_trajectories = new HashMap<String, Trajectory>();

    /**
     * The constructor for the TrajectorBuilder class takes a path to the trajectory JSON
     * directory as input. This is relative to the current deploy directory so, if the
     * paths are in home/lvuser/deploy/paths, you would pass "paths".
     */
    public TrajectoryBuilder(String PathSubDir)
    {
        Timer timer = new Timer();

        File deployDirectory = Filesystem.getDeployDirectory();
        File pathDirectory = new File(deployDirectory, PathSubDir);

        File[] pathNames = pathDirectory.listFiles();

        timer.stop();
        timer.reset();
        timer.start();

        for (File pathname : pathNames)
        {
            // Only consider files which appear to be JSON.
            if (!pathname.getName().contains(".json"))
            {
                continue;
            }

            // Creating trajectory with json
            Trajectory trajectory = makeTrajectoryFromJSON(pathname);
            if (trajectory != null)
            {
                // Add trajectory to set
                m_trajectories.put(pathname.getName().replace(".wpilib.json", ""), trajectory);
            }
        }
    }

    /**
     * Gets the number of trajectories available
     * 
     * @return The number of treajectories
     */
    public int getNumTrajectories()
    {
        return m_trajectories.size();
    }

    /**
     * Gets the names of the built trajectories
     * 
     * @return A set with all the trajectory names
     */
    public Set<String> getTrajectoryNames()
    {
        return m_trajectories.keySet();
    }

    /**
     * Gets the specified trajectory by name
     * 
     * @param name
     *            The name of the desired trajectory
     * @return A trajectory with the specified name
     */
    public Trajectory getTrajectory(String name)
    {
        return m_trajectories.get(name);
    }

    /**
     * Checks if the specified trajectory exists
     * 
     * @param name
     *            The name of the specified trajectory
     * @return Whether the trajectory exists
     */
    public boolean hasTrajectory(String name)
    {
        return m_trajectories.containsKey(name);
    }

    /**
     * Checks if all of the specified trajectories exist. Can be used to see if all the
     * required trajectories needed to make a auto command exists.
     * 
     * @param names
     *            A string array with all of the desired trajectories
     * @return Whether all of the specified trajectories exist
     */
    // TODO: Check if this works as expected
    public boolean hasTrajectories(String[] names)
    {
        for (String name : names)
        {
            if (!hasTrajectory(name))
            {
                return false;
            }
        }
        return true;
    }

    /**
     * Checks if all of the specified trajectories exist. Can be used to see if all the
     * required trajectories needed to make a auto command exists.
     * 
     * @param names
     *            A string set with all of the desired trajectories
     * @return Whether all of the specified trajectories exist
     */
    // TODO: Check if this works as expected
    public boolean hasTrajectories(Set<String> names)
    {
        for (String name : names)
        {
            if (!hasTrajectory(name))
            {
                return false;
            }
        }
        return true;
    }

    /**
     * Creates a trajectory using a Pathweaver-style json file
     * 
     * @param trajectoryJSON
     *            The desire json file to create a trajectory from
     * @return A trajectory made from the json
     */
    private Trajectory makeTrajectoryFromJSON(File trajectoryJSON)
    {
        Trajectory trajectory = new Trajectory();
        try
        {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryJSON.toPath());
        }
        catch (IOException e)
        {
            // If we are unable to open the file the method returns a null object
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON.getName(),
                e.getStackTrace());
            return null;
        }
        return trajectory;
    }
}
