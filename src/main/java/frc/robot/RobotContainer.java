/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.Set;

import edu.wpi.first.cameraserver.CameraServer;

//import com.fasterxml.jackson.core.JsonFactory;
//import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultTankDriveCommand;
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
        DoNothing, DriveSplineFromJSON, DriveSplineCanned, Drive1mForwardBackward
    };

    private final TrajectoryBuilder trajectoryCreator = new TrajectoryBuilder(Constants.SPLINE_DIRECTORY);
    private SendableChooser<AutoCommands> autoCommandSelector = new SendableChooser<AutoCommands>();
    private SendableChooser<Command> driveModeSelector = new SendableChooser<Command>();

    private SendableChooser<Trajectory> splineCommandSelector = new SendableChooser<Trajectory>();

    // The Robot controllers
    private final FilteredJoystick driverLeftJoystick = new FilteredJoystick(Ports.OIDriverLeftJoystick);
    private final FilteredJoystick driverRightJoystick = new FilteredJoystick(Ports.OIDriverRightJoystick);
  
    // The robot's subsystems are defined here...
    private final SK21Drive driveSubsystem = new SK21Drive();

    private final DefaultDriveCommand arcadeDrive = new DefaultDriveCommand(driveSubsystem, driverLeftJoystick);
    private final DefaultTankDriveCommand tankDrive 
                                    = new DefaultTankDriveCommand(driveSubsystem, driverLeftJoystick, driverRightJoystick);
  
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

    public void resetDriveDefaultCommand()
    {
         // Configure default commands
         // Set the default drive command to split-stick arcade drive
         driveSubsystem.setDefaultCommand(driveModeSelector.getSelected());
    }

    private void configureShuffleboard()
    {
        // auto commands
        autoCommandSelector.setDefaultOption("Do Nothing", AutoCommands.DoNothing);
        autoCommandSelector.addOption("Drive path from JSON", AutoCommands.DriveSplineFromJSON);
        autoCommandSelector.addOption("Drive canned path", AutoCommands.DriveSplineCanned);
        autoCommandSelector.addOption("Drive forwards then backwards 1m", AutoCommands.Drive1mForwardBackward);

        driveModeSelector.setDefaultOption("Arcade Drive", arcadeDrive);
        driveModeSelector.addOption("Tank Drive", tankDrive);

        Set<String> splineDirectory = trajectoryCreator.getTrajectoryNames();

        for (String pathname : splineDirectory)
        {
            splineCommandSelector.addOption(pathname, trajectoryCreator.getTrajectory(pathname));
        }

        SmartDashboard.putData("Auto Chooser", autoCommandSelector);
        SmartDashboard.putData("Drive Mode", driveModeSelector);
        SmartDashboard.putData("JSON Selector", splineCommandSelector);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Turns on slowmode when driver presses slowmode button, giving more manueverability.
        new JoystickButton(driverLeftJoystick, Ports.OIDriverSlowmode)
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
        File splineDirectory = new File(deployDirectory, Constants.SPLINE_DIRECTORY);

        var autoSelector = autoCommandSelector.getSelected();

        switch (autoSelector)
        {
            case DoNothing:
                return new DoNothingCommand();

            case DriveSplineFromJSON:
                // Note that the drive constraints are baked into the PathWeaver output so they are not
                // mentioned here.
                Trajectory trajectory = splineCommandSelector.getSelected();
                if (trajectory == null)
                {
                    return new DoNothingCommand();
                }
                return makeTrajectoryCommand(trajectory, true);
            
            // This sequentially runs thorugh the 2 sub-paths of the Drive1mForwardBackward path defined in PathWeaver 
            case Drive1mForwardBackward:

                // Required segments for the path
                String[] dependencies = {"1m Forwards", "1m Backwards"};
                // Check if each required segment has been created
                if(trajectoryCreator.hasTrajectories(dependencies))
                {
                    // Create commands for each segment
                    Command drive1mfCommand = makeTrajectoryCommand(trajectoryCreator.getTrajectory("1m Forwards"), true);
                    Command drive1mbCommand = makeTrajectoryCommand(trajectoryCreator.getTrajectory("1m Backwards"), false);
                    // Execute each of the single commands in chronological order
                    return new SequentialCommandGroup(drive1mfCommand, drive1mbCommand);
                }
                else
                {
                    return new DoNothingCommand();
                }

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
                        new RamseteController(AutoConstants.RAMSETE_B,
                                              AutoConstants.RAMSETE_ZETA),
                        new SimpleMotorFeedforward(DriveConstants.KS,
                                                   DriveConstants.KV,
                                                   DriveConstants.KA),
                                                   DriveConstants.DRIVE_KINEMATICS, driveSubsystem::getWheelSpeeds,
                        new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0),
                        new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0),
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
