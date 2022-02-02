/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import java.util.Set;

import com.ctre.phoenix.schedulers.SequentialScheduler;

import edu.wpi.first.cameraserver.CameraServer;

//import com.fasterxml.jackson.core.JsonFactory;
//import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultTankDriveCommand;
import frc.robot.commands.DoNothingCommand;
import frc.robot.subsystems.SK21Drive;
import frc.robot.subsystems.base.SuperClasses.AutoCommands;
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
    private UsbCamera camera1;
    private UsbCamera camera2;
    NetworkTableEntry cameraSelection;

    

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
            camera1 = CameraServer.startAutomaticCapture("Driver Front Camera", 0);
            camera1.setResolution(240, 240);
            camera1.setFPS(15);

            camera2 = CameraServer.startAutomaticCapture("Driver Rear Camera", 1);
            camera2.setResolution(240, 240);
            camera2.setFPS(15);

            cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

            // to change camera displayed feed later on, use the following code
            // cameraSelection.setString(camera2.getName());
            // cameraSelection.setString(camera1.getName());
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
        addPossibleAutos();

        driveModeSelector.setDefaultOption("Arcade Drive", arcadeDrive);
        driveModeSelector.addOption("Tank Drive", tankDrive);

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


    /**
     * Adds the possible auto commands to the Shuffleboard list depending on the
     * auto segments that are present according to the {@link TrajectoryBuilder}.
     * Adds the auto commands to their respective {@link SendableChooser} to be
     * used by {@link SmartDashboard}.
     */
    public void addPossibleAutos()
    {
        // Adding Segment Independent Paths
        autoCommandSelector.setDefaultOption("Do Nothing", AutoCommands.DoNothing);
        autoCommandSelector.addOption("Drive path from JSON", AutoCommands.DriveSplineFromJSON);
        autoCommandSelector.addOption("Drive canned path", AutoCommands.DriveSplineCanned);

        /*
            Checking dependencies for autos before giving option to run
        */

        // Simple paths
        if(trajectoryCreator.hasTrajectories(new String[]{"1m Forwards", "1m Backwards"}))
        {
            autoCommandSelector.addOption("Drive forwards then backwards 1m", AutoCommands.Drive1mForwardBackward);
        }
        if(trajectoryCreator.hasTrajectory("Simple Taxi"))
        {
            autoCommandSelector.addOption("Taxi", AutoCommands.Taxi);
        }

        // Autos that start at the hub (Tarmac 1A)
        if(trajectoryCreator.hasTrajectory("Low to Ball 1 (LH)"))
        {
            autoCommandSelector.addOption("Low to Ball 2", AutoCommands.N2_LH_1A);
            if(trajectoryCreator.hasTrajectory("Ball 1 to Low"))
            {
                autoCommandSelector.addOption("2 Ball Tarmac 1A", AutoCommands.N2_LL_1A);
            }
            if(trajectoryCreator.hasTrajectory("Ball 1 to Ball 2"))
            {
                autoCommandSelector.addOption("3 Ball Tarmac 1A", AutoCommands.N3_LHH_2B);
            }
        }
        // Autos that start at the hub (Tarmac 2A)
        if(trajectoryCreator.hasTrajectory("Low to Ball 2 (LH)"))
        {
            autoCommandSelector.addOption("Low to Ball 2", AutoCommands.N2_LH_2A);
            if(trajectoryCreator.hasTrajectory("Ball 2 to Low"))
            {
                autoCommandSelector.addOption("2 Ball Tarmac 2A", AutoCommands.N2_LL_2A);
            }
            // TODO: Look at extra option (3 Ball PreLoaded-2-1)
            if(trajectoryCreator.hasTrajectory("Ball 2 to Ball 3"))
            {
                autoCommandSelector.addOption("3 Ball Tarmac 2A", AutoCommands.N3_LHH_2B);
            }
        }
        // Autos that start at the hub (Tarmac 2B)
        if(trajectoryCreator.hasTrajectory("Low to Ball 3 (LH)"))
        {
            autoCommandSelector.addOption("Low to Ball 3", AutoCommands.N2_LH_2B);
            if(trajectoryCreator.hasTrajectory("Ball 3 to Low"))
            {
                autoCommandSelector.addOption("2 Ball Tarmac 2B", AutoCommands.N2_LL_2B);
            }
            if(trajectoryCreator.hasTrajectory("Ball 3 to Ball 2"))
            {
                autoCommandSelector.addOption("3 Ball Tarmac 2B", AutoCommands.N3_LHH_2B);
            }
        }

        // Paths that require the terminal ball
        if(trajectoryCreator.hasTrajectory("Terminal to Shoot"))
        {
            if(trajectoryCreator.hasTrajectory("Ball 2 to Terminal"))
            {
                if(trajectoryCreator.hasTrajectory("Grab Ball Radial (HH)"))
                {
                    if(trajectoryCreator.hasTrajectory("Ball 1 to Ball 2"))
                    {
                        autoCommandSelector.addOption("4T Ball Radial Tarmac 1A HHHH", AutoCommands.T4_HHHH_R1A);
                    }
                    if(trajectoryCreator.hasTrajectory("Ball 3 to Ball 2"))
                    {
                        autoCommandSelector.addOption("4T Ball Radial Tarmac 2B HHHH", AutoCommands.T4_HHHH_R2B);
                    }
                    autoCommandSelector.addOption("2 Ball Radial HH", AutoCommands.N2_HH_R);
                }
                if(trajectoryCreator.hasTrajectories(new String[]{"Low to Ball 1 (LH)", "Ball 1 to Ball 2"}))
                {
                    autoCommandSelector.addOption("4 Ball Tarmac 1A", AutoCommands.T4_LHHH_1A);
                }
                if(trajectoryCreator.hasTrajectories(new String[]{"Low to Ball 3 (LH)", "Ball 3 to Ball 2"}))
                {
                    autoCommandSelector.addOption("4 Ball Tarmac 2B", AutoCommands.T4_LHHH_2B);
                }
            }
        }

        // Non-Terminal Ball Autos that Start Radially
        if(trajectoryCreator.hasTrajectory("Grab Ball Radial (HH)"))
        {
            if(trajectoryCreator.hasTrajectory("Ball 2 to Ball 3"))
            {
                autoCommandSelector.addOption("3 Ball Tarmac R1A", AutoCommands.N3_LHH_2B);
            }
            if(trajectoryCreator.hasTrajectory("Ball 2 to Ball 3"))
            {
                autoCommandSelector.addOption("3 Ball Tarmac R2A", AutoCommands.N3_LHH_2B);
            }
            if(trajectoryCreator.hasTrajectory("Ball 3 to Ball 2"))
            {
                autoCommandSelector.addOption("3 Ball Tarmac R2B", AutoCommands.N3_LHH_2B);
            }
        }

        // Adding all JSON paths
        Set<String> splineDirectory = trajectoryCreator.getTrajectoryNames();
        for (String pathname : splineDirectory)
        {
            splineCommandSelector.addOption(pathname, trajectoryCreator.getTrajectory(pathname));
        }
        String firstJSON = splineDirectory.stream().findFirst().get();
        splineCommandSelector.setDefaultOption(firstJSON, trajectoryCreator.getTrajectory(firstJSON));
    }

    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
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
            
            case DriveSplineCanned:
                // Create a voltage constraint to ensure we don't accelerate too fast
                var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV,
                        DriveConstants.KS),
                    DriveConstants.DRIVE_KINEMATICS, 10);
                TrajectoryConfig config = new TrajectoryConfig(AutoConstants.MAX_SPEED,
                    AutoConstants.MAX_ACCELERATION)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.DRIVE_KINEMATICS)
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
                // Execute each of the single commands in chronological order
                return new SequentialCommandGroup(
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("1m Forwards"), true), 
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("1m Backwards"), false));

            case Taxi:
                return makeTrajectoryCommand(trajectoryCreator.getTrajectory("Simple Taxi"), true);
            
            case N2_HH_R:
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        makeTrajectoryCommand(trajectoryCreator.getTrajectory("Grab Ball Radial (HH)"), true),
                        new DoNothingCommand()),    // Set Up Intake
                    new DoNothingCommand());        // Launcher Shoot HH

            case N2_LL_1A:
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new DoNothingCommand(),     // Launcher Shoot L
                        new DoNothingCommand()),    // Set Up Intake
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Low to Ball 1 (LH)"), true),
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Ball 1 to Low"), false),
                    new DoNothingCommand());        // Launcher Shoot L
                
            case N2_LL_2A:
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new DoNothingCommand(),     // Launcher Shoot L
                        new DoNothingCommand()),    // Set Up Intake
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Low to Ball 2 (LH)"), true),
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Ball 2 to Low"), false),
                    new DoNothingCommand());         // Launcher Shoot L

            case N2_LL_2B:
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new DoNothingCommand(),     // Launcher Shoot L
                        new DoNothingCommand()),    // Set Up Intake
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Low to Ball 3 (LH)"), true),
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Ball 3 to Low"), false),
                    new DoNothingCommand());        // Launcher Shoot L
            
            case N2_LH_1A:
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new DoNothingCommand(),     // Launcher Shoot L
                        new DoNothingCommand()),    // Set Up Intake
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Low To Ball 1 (LH)"), true),
                    new DoNothingCommand());        // Launcher Shoot H

            case N2_LH_2A:
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new DoNothingCommand(),     // Launcher Shoot L
                        new DoNothingCommand()),    // Set Up Intake
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Low To Ball 2 (LH)"), true),
                    new DoNothingCommand());        // Launcher Shoot H
                
            case N2_LH_2B:
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new DoNothingCommand(),     // Launcher Shoot L
                        new DoNothingCommand()),    // Set Up Intake
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Low To Ball 3 (LH)"), true),
                    new DoNothingCommand());        // Launcher Shoot H
                

            case T4_LHHH_1A:
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new DoNothingCommand(),     // Launcher Shoot L
                        new DoNothingCommand()),    // Set Up Intake
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Low to Ball 1 (LH)"), true),
                    new DoNothingCommand(),         // Launcher Shoot H
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Ball 1 to Ball 2"), false),
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Ball 2 to Terminal"), false),
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Terminal to Shoot"), false),
                    new DoNothingCommand());        // Launcher Shoot HH

            case T4_LHHH_2B:
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new DoNothingCommand(),     // Launcher Shoot L
                        new DoNothingCommand()),    // Set Up Intake
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Low to Ball 3 (LH)"), true),
                    new DoNothingCommand(),         // Launcher Shoot H
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Ball 3 to Ball 2"), false),
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Ball 2 to Terminal"), false),
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Terminal to Shoot"), false),
                    new DoNothingCommand());        // Launcher Shoot HH

            case T4_HHHH_R1A:
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        makeTrajectoryCommand(trajectoryCreator.getTrajectory("Grab Ball Radial (HH)"), true),
                        new DoNothingCommand(),     // Intake Set Up
                        new DoNothingCommand()),    // Launcher Set Up
                    new DoNothingCommand(),         // Launcher Shoot HH
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Ball 1 to Ball 2"), false),
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Ball 2 to Terminal"), false),
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Terminal to Shoot"), false),
                    new DoNothingCommand());        // Launcher Shoot HH

            case T4_HHHH_R2B:
                return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        makeTrajectoryCommand(trajectoryCreator.getTrajectory("Grab Ball Radial (HH)"), true),
                        new DoNothingCommand(),     // Intake Set Up
                        new DoNothingCommand()),    // Launcher Set Up
                    new DoNothingCommand(),         // Launcher Shoot HH
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Ball 3 to Ball 2"), false),
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Ball 2 to Terminal"), false),
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Terminal to Shoot"), false),
                    new DoNothingCommand());        // Launcher Shoot HH

            default:
                DriverStation.reportError("Uncoded selection from autoSelector chooser!", false);
                return new DoNothingCommand();

        }
    }
}
