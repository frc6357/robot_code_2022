package frc.robot.AutoTools.SK22Paths;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.DifferentialDrivetrain;
import frc.robot.AutoTools.AutoPaths;
import frc.robot.AutoTools.RamseteTrajectoryMethod;
import frc.robot.AutoTools.TrajectoryBuilder;

/**
 * A class that contains an auto path that is written using Pose2D and used for testing
 * purposes
 */
public class DriveSplineCanned implements AutoPaths
{
    /**
     * A function that gets the name of the auto command
     * 
     * @return The name of the command
     */
    public String getName()
    {
        return "Drive Canned Path";
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
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            AutoConstants.SIMPLE_MOTOR_FEEDFORWARD, DriveConstants.DRIVE_KINEMATICS, 10);

        TrajectoryConfig config =
                new TrajectoryConfig(AutoConstants.MAX_SPEED, AutoConstants.MAX_ACCELERATION)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(DriveConstants.DRIVE_KINEMATICS)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(0.5, 0)),
            new Pose2d(1, 0, new Rotation2d(0)), config);

        return trajectoryCreator.createTrajectory(trajectory, true, driveSubsystem);
    }
}
