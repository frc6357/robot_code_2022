/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.List;
//import java.util.Optional;

//import com.fasterxml.jackson.core.JsonFactory;
//import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DoNothingCommand;
import frc.robot.subsystems.SK21Drive;
import frc.robot.utils.FilteredJoystick;
//import frc.robot.utils.SubsystemControls;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer
{

    /**
     * The USB Camera for the Robot.
     */
    private UsbCamera camera;

    /**
     * Available autonomous commands for the Robot.
     */
    private enum AutoCommands
    {
        DoNothing, DriveSplineFromJSON, DriveSplineCanned, Drive1mForwardBackward, DriveBounce
    };

    private SendableChooser<AutoCommands> autoCommandSelector = new SendableChooser<AutoCommands>();

    private SendableChooser<File> splineCommandSelector = new SendableChooser<File>();

    // The Robot controllers
    private final FilteredJoystick driverJoystick = new FilteredJoystick(0);
  
    // The robot's subsystems are defined here...
    private final SK21Drive driveSubsystem = new SK21Drive();
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
    public RobotContainer()
    {
        configureShuffleboard();

        // TODO: Add code here to load optional subsystems. This is left as an example.
        //
        // File deployDirectory = Filesystem.getDeployDirectory();
        // File subsystemFile = new File(deployDirectory, Constants.kSubsystem);
        //
        // ObjectMapper mapper = new ObjectMapper();
        // JsonFactory factory = new JsonFactory();
        //
        // try
        // {
        //     JsonParser parser = factory.createParser(subsystemFile);
        //     SubsystemControls subsystems = mapper.readValue(parser, SubsystemControls.class);
        //  
        //    if (subsystems.isLauncherPresent())
        //    {
        //        launcherSubsystem  = Optional.of(new SK21Launcher());
        //    }
        // }
        // catch (IOException e)
        // {
        //     DriverStation.reportError("Failure to read Subsystem Control File!", e.getStackTrace());
        // }

        // Configure the button bindings
        configureButtonBindings();

        resetDriveDefaultCommand();

        // Driver camera configuration.
        if (RobotBase.isReal())
        {
            camera = CameraServer.startAutomaticCapture("Driver Front Camera", 0);
            camera.setResolution(240, 240);
            camera.setFPS(15);
        }
    }

    private void resetDriveDefaultCommand()
    {
         // Configure default commands
         // Set the default drive command to split-stick arcade drive
         driveSubsystem.setDefaultCommand(new DefaultDriveCommand(driveSubsystem, driverJoystick));
    }

    private void configureShuffleboard()
    {
        // auto commands
        autoCommandSelector.setDefaultOption("Do Nothing", AutoCommands.DoNothing);
        autoCommandSelector.addOption("Drive path from JSON", AutoCommands.DriveSplineFromJSON);
        autoCommandSelector.addOption("Drive canned path", AutoCommands.DriveSplineCanned);
        autoCommandSelector.addOption("Drive forwards then backwards 1m", AutoCommands.Drive1mForwardBackward);
        autoCommandSelector.addOption("Drive bounce path", AutoCommands.DriveBounce);
    
        SmartDashboard.putData("Auto Chooser", autoCommandSelector);

        File deployDirectory = Filesystem.getDeployDirectory();
        File splineDirectory = new File(deployDirectory, Constants.kSplineDirectory);

        File[] pathNames = splineDirectory.listFiles();
        for (File pathname : pathNames)
        {
            // Print the names of files and directories
            System.out.println(pathname);
            splineCommandSelector.addOption(pathname.getName(), pathname);
        }

        SmartDashboard.putData(splineCommandSelector);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // TODO: Is the following control in the robot driver user interface specification?
        // TODO: Test this implementation to make sure that it works as expected since it's 
        // different from the way we've done this in the past.
        new JoystickButton(driverJoystick, Button.kRightBumper.value)
            .whenPressed(() -> driveSubsystem.setMaxOutput(0.5))
            .whenReleased(() -> driveSubsystem.setMaxOutput(1));

        // TODO: Left as an example of how to do optional subsystem button binding.
        //Launcher
        // if (launcherSubsystem.isPresent())
        // {
        //     var launcher = launcherSubsystem.get();
        //     setHighAngle.whenPressed(new SetHoodHighShotCommand(launcher));
        //     setLowAngle.whenPressed(new SetHoodLowShotCommand(launcher));
        //     toggleLauncherSpeed.whenPressed(new LauncherSpeedCommand(launcher));
        // }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        File deployDirectory = Filesystem.getDeployDirectory();
        File splineDirectory = new File(deployDirectory, Constants.kSplineDirectory);

        var autoSelector = autoCommandSelector.getSelected();

        switch (autoSelector)
        {
            case DoNothing:
                return new DoNothingCommand();

            case DriveSplineFromJSON:
                // Note that the drive constraints are baked into the PathWeaver output so they are not
                // mentioned here.
                File splineFile = splineCommandSelector.getSelected();
                Trajectory trajectory = makeTrajectoryFromJSON(splineFile);
                if (trajectory == null)
                {
                    return new DoNothingCommand();
                }
                return makeTrajectoryCommand(trajectory, true);
            
            case DriveSplineCanned:
                // Create a voltage constraint to ensure we don't accelerate too fast
                var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                    DriveConstants.kDriveKinematics, 10);
                TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
                // Trajectory cannedTrajectory = TrajectoryGenerator.generateTrajectory(
                //                 new Pose2d(0, 0, new Rotation2d(0)),
                //                 List.of(new Translation2d(2, 1), new Translation2d(3, -1)),
                //                 new Pose2d(5, 0, new Rotation2d(0)), config);
                Trajectory cannedTrajectory = 
                TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), 
                    List.of(new Translation2d(0.5, 0)),
                    new Pose2d(1, 0, new Rotation2d(0)), config);

                return makeTrajectoryCommand(cannedTrajectory, true);
            
            // This sequentially runs thorugh the 2 sub-paths of the Drive1mForwardBackward path defined in PathWeaver 
            case Drive1mForwardBackward:
                
                // Generate a command for driving 1m forward from trajectory created from PathWeaver JSON file
                File drive1mf = new File(splineDirectory + "/1m Forwards.wpilib.json");
                Trajectory drive1mfTrajectory = makeTrajectoryFromJSON(drive1mf);
                if (drive1mfTrajectory == null)
                {
                    return new DoNothingCommand();
                }
                Command drive1mfCommand = makeTrajectoryCommand(drive1mfTrajectory, true);

                // Generate a command for driving 1m backward from trajectory created from PathWeaver JSON file
                File drive1mb = new File(splineDirectory + "/1m Backwards.wpilib.json");
                Trajectory drive1mbTrajectory = makeTrajectoryFromJSON(drive1mb);
                if (drive1mbTrajectory == null)
                {
                    return new DoNothingCommand();
                }
                Command drive1mbCommand = makeTrajectoryCommand(drive1mbTrajectory, false);

                // Execute each of the single commands in chronological order
                return new SequentialCommandGroup(drive1mfCommand, drive1mbCommand);
            
            // This sequentially runs thorugh the 4 sub-paths of the Bounce Path defined in PathWeaver  
            case DriveBounce:
                
                // Generate a command for driving the 1st segment Bounce Path trajectory defined by PathWeaver JSON file
                File bounceSeg1 = new File(splineDirectory + "/Bounce Segment 1.wpilib.json");
                Trajectory bounceSeg1Trajectory = makeTrajectoryFromJSON(bounceSeg1);
                if (bounceSeg1Trajectory == null)
                {
                    return new DoNothingCommand();
                }
                Command driveBounceSeg1 = makeTrajectoryCommand(bounceSeg1Trajectory, true);

                // Generate a command for driving the 2nd segment Bounce Path trajectory defined by PathWeaver JSON file
                File bounceSeg2 = new File(splineDirectory + "/Bounce Segment 2.wpilib.json");
                Trajectory bounceSeg2Trajectory = makeTrajectoryFromJSON(bounceSeg2);
                if (bounceSeg2Trajectory == null) 
                {
                    return new DoNothingCommand();
                }
                Command driveBounceSeg2 = makeTrajectoryCommand(bounceSeg2Trajectory, false);

                // Generate a command for driving the 3rd segment Bounce Path trajectory defined by PathWeaver JSON file
                File bounceSeg3 = new File(splineDirectory + "/Bounce Segment 3.wpilib.json");
                Trajectory bounceSeg3Trajectory = makeTrajectoryFromJSON(bounceSeg3);
                if (bounceSeg3Trajectory == null)
                {
                    return new DoNothingCommand();
                }
                Command driveBounceSeg3 = makeTrajectoryCommand(bounceSeg3Trajectory, false);
                
                // Generate a command for driving the 4th segment Bounce Path trajectory defined by PathWeaver JSON file
                File bounceSeg4 = new File(splineDirectory + "/Bounce Segment 4.wpilib.json");
                Trajectory bounceSeg4Trajectory = makeTrajectoryFromJSON(bounceSeg4);
                if (bounceSeg4Trajectory == null)
                {
                    return new DoNothingCommand();
                }
                Command driveBounceSeg4 = makeTrajectoryCommand(bounceSeg4Trajectory, false);

                // Execute each of the single commands in chronological order
                return new SequentialCommandGroup(driveBounceSeg1, driveBounceSeg2, driveBounceSeg3, driveBounceSeg4);

            default:
                DriverStation.reportError("Uncoded selection from autoSelector chooser!", false);
                return new DoNothingCommand();

        }
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

    private Command makeTrajectoryCommand(Trajectory trajectory, boolean bFirst) 
    {
        RamseteCommand ramseteCommand = new RamseteCommand(trajectory,
                                                           driveSubsystem::getPose,
                        new RamseteController(AutoConstants.kRamseteB,
                                              AutoConstants.kRamseteZeta),
                        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                                   DriveConstants.kvVoltSecondsPerMeter,
                                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
                                                   DriveConstants.kDriveKinematics, driveSubsystem::getWheelSpeeds,
                        new PIDController(DriveConstants.kPDriveVel, 0, 0),
                        new PIDController(DriveConstants.kPDriveVel, 0, 0),
                        // RamseteCommand passes volts to the callback
                        driveSubsystem::tankDriveVolts, driveSubsystem);
    
        // Tell the robot where it is starting from if this is the first trajectory of a path.
        if (bFirst)
        {
            driveSubsystem.resetOdometry(trajectory.getInitialPose());
        }

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> driveSubsystem.tankDriveVolts(0, 0));
    }

    /**
     * Reset the encoders and gyro in the drive subsystem. This should be called
     * on boot and when initializing auto and reset modes.
     */
    public void resetDriveSubsystem()
    {
        driveSubsystem.resetEncoders();
        driveSubsystem.resetGyro();
    }
}
