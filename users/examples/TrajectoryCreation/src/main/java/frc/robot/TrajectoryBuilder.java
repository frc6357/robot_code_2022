// A class used to pre-build trajectories from JSON files placed in the 
// deploy/paths directory of the RoboRio. This is invoked from robot_init
// to ensure that all the time-consuming trajectory creation is done well
// before autonomous mode starts.

package frc.robot;

import java.io.File;
import java.io.IOException;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Timer;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class TrajectoryBuilder {

    private Map<String, Trajectory> m_trajectories = new HashMap<String, Trajectory>();

    /** The constructor for the TrajectorBuilder class takes a path to the trajectory JSON
    * directory as input. This is relative to the current deploy directory so, if the 
    * paths are in home/lvuser/deploy/paths, you would pass "paths".
    */
    public TrajectoryBuilder(String PathSubDir)
    {
        Timer timer = new Timer();
        double time;
        double last_time = 0.0;

        File deployDirectory = Filesystem.getDeployDirectory();
        File pathDirectory = new File(deployDirectory, PathSubDir);

        File[] pathNames = pathDirectory.listFiles();

        System.out.println("Building auto trajectories...");

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

            System.out.printf("  %s\n", pathname.getName());

            Trajectory trajectory = makeTrajectoryFromJSON(pathname);
            if (trajectory == null)
            {
                System.out.printf("    Failure building trajectory from JSON file %s!\n", pathname.getName());
            }
            else
            {
                m_trajectories.put(pathname.getName(), trajectory);
            }

            time = timer.get();
            System.out.printf("%f (%f) Trajectory %s created.\n", time-last_time, time, pathname.getName());
            last_time = time;
        }

        System.out.printf("Successfully build %d trajectories.\n", m_trajectories.size());
    }

    public int getNumTrajectories()
    {
        return m_trajectories.size();
    }

    public Set<String> getTrajectoryNames()
    {
        return m_trajectories.keySet();
    }

    public Trajectory getTrajectory(String name)
    {
        return m_trajectories.get(name);
    }

    public boolean hasTrajectory(String name)
    {
        return m_trajectories.containsKey(name);
    }

    private Trajectory makeTrajectoryFromJSON(File trajectoryJSON)
    {
        Trajectory trajectory = new Trajectory();
        try
        {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryJSON.toPath());
        }
        catch (IOException ex)
        {
            // If we are unable to open the file the method returns a null object
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON.getName(), ex.getStackTrace());
            return null;
        }
        return trajectory;
    }
}
