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
import java.util.Optional;
import java.util.Set;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.revrobotics.CANSparkMax;

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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AcquireTargetCommand;
import frc.robot.commands.DefaultArcadeDriveCommand;
import frc.robot.commands.DefaultTankDriveCommand;
import frc.robot.commands.DoNothingCommand;
import frc.robot.commands.SetIntakePositionCommand;
import frc.robot.commands.ShootBallsCommand;
import frc.robot.subsystems.SK22Climb;
import frc.robot.subsystems.SK22Drive;
import frc.robot.subsystems.SK22Intake;
import frc.robot.subsystems.SK22Launcher;
import frc.robot.subsystems.SK22Transfer;
import frc.robot.subsystems.SK22Vision;
import frc.robot.subsystems.base.TriggerButton;
import frc.robot.subsystems.base.SuperClasses.AutoCommands;
import frc.robot.subsystems.base.SuperClasses.Gear;
import frc.robot.utils.FilteredJoystick;
import frc.robot.utils.SK22CommandBuilder;
import frc.robot.utils.SubsystemControls;
//import frc.robot.utils.SubsystemControls;
import frc.robot.utils.TrajectoryBuilder;

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
    private final SK22CommandBuilder pathBuilder = 
                            new SK22CommandBuilder(Constants.AUTOS_FOLDER_DIRECTORY, trajectoryCreator);
    private SendableChooser<AutoCommands> autoCommandSelector = new SendableChooser<AutoCommands>();
    private SendableChooser<Command> driveModeSelector = new SendableChooser<Command>();

    private SendableChooser<Trajectory> splineCommandSelector = new SendableChooser<Trajectory>();

    // The Robot controllers
    private final FilteredJoystick driverLeftJoystick = new FilteredJoystick(Ports.OIDriverLeftJoystick);
    private final FilteredJoystick driverRightJoystick = new FilteredJoystick(Ports.OIDriverRightJoystick);
  
    // The robot's subsystems are defined here...
    // TODO: Find which one is high gear and which one is low gear
    private final SK22Drive driveSubsystem = new SK22Drive(
                                                new DoubleSolenoid(
                                                    Ports.BASE_PCM, 
                                                    Ports.pneumaticsModuleType, 
                                                    Ports.gearShiftHigh, 
                                                    Ports.gearShiftLow));
    // These are currently empty and only created in the contructor
    // based on the Subsystem.json file
    private Optional<SK22Intake> intakeSubsystem = Optional.empty();
    private Optional<SK22Launcher> launcherSubsystem = Optional.empty();
    private Optional<SK22Transfer> transferSubsystem = Optional.empty();
    private final Optional<SK22Climb> climbSubsystem;
    private Optional<SK22Vision> visionSubsystem = Optional.empty();

    // Robot External Controllers (Joysticks and Logitech Controller)
    private final Joystick operatorJoystick = new Joystick(Ports.OIOperatorController);

    // Transfer control buttons
    private final JoystickButton ejectBall = new JoystickButton(operatorJoystick, Ports.OIOperatorEjectBallButton);
    // Vision control buttons
    // TODO: Figure out how to use left and right joystick if we are using both left and right joysticks
    // as the buttons could go on either joystick. Using arcade drive could result this button to be
    // put on another joystick compared to tank drive.
    private final JoystickButton acquireTarget = new JoystickButton(driverLeftJoystick, Ports.OIDriverAcquireTarget);

    private final DefaultArcadeDriveCommand arcadeDrive
                                = new DefaultArcadeDriveCommand(driveSubsystem, driverLeftJoystick);
    private final DefaultTankDriveCommand tankDrive 
                                = new DefaultTankDriveCommand(driveSubsystem, driverLeftJoystick, driverRightJoystick);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
    public RobotContainer(Optional<SK22Climb> climbSubsystem)
    {
        this.climbSubsystem = climbSubsystem;
    
        configureShuffleboard();

        File deployDirectory = Filesystem.getDeployDirectory();
        
        ObjectMapper mapper = new ObjectMapper();
        JsonFactory factory = new JsonFactory();
        
        try
        {
            // Looking for the Subsystems.json file in the deploy directory
            JsonParser parser = factory.createParser(new File(deployDirectory, Constants.SUBSYSTEM));
            SubsystemControls subsystems = mapper.readValue(parser, SubsystemControls.class);
         
            // Instantiating subsystems if they are present
            // This is decided by looking at Subsystems.json
            if(subsystems.isIntakePresent())
            {
                intakeSubsystem = Optional.of(new SK22Intake());
            }
            if(subsystems.isLauncherPresent())
            {
                launcherSubsystem  = Optional.of(new SK22Launcher());
            }
            if(subsystems.isTransferPresent())
            {
                transferSubsystem  = Optional.of(new SK22Transfer());
            }
            if(subsystems.isVisionPresent())
            {
                visionSubsystem  = Optional.of(new SK22Vision());
            }
        }
        catch (IOException e)
        {
            DriverStation.reportError("Failure to read Subsystem Control File!", e.getStackTrace());
        }

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
        
        // Sets the gear to low when driver clicks setLowGear Buttons
        new JoystickButton(driverLeftJoystick, Ports.OIDriverSetLowGear)
            .whenPressed(() -> driveSubsystem.setGear(Gear.LOW));

        // Sets the gear to high when driver clicks setHighGear Buttons
        new JoystickButton(driverLeftJoystick, Ports.OIDriverSetHighGear)
            .whenPressed(() -> driveSubsystem.setGear(Gear.HIGH));

        // Set up intake buttons if the intake is present
        if(intakeSubsystem.isPresent())
        {
            SK22Intake intake = intakeSubsystem.get();
            
            // Extends the intake when the extendIntake Button is pressed
            new JoystickButton(operatorJoystick, Ports.OIOperatorIntakeExtend)
                .whenPressed(new SetIntakePositionCommand(intake, true));
            // Retracts the intake when the retractIntake Button is pressed
            new JoystickButton(operatorJoystick, Ports.OIOperatorIntakeRetract)
                .whenPressed(new SetIntakePositionCommand(intake, false));
            
            // TODO: Check if we would rather set the intake command using the method
            // This is only viable if the intake extension and retraction do not severely
            // increase in complexity
            // shown below or using the method shown above.
            // // Extends the intake when the extendIntake Button is pressed
            // new JoystickButton(operatorJoystick, Ports.OIOperatorIntakeExtend)
            //     .whenPressed(() -> 
            //         {intake.extendIntake();
            //         intake.setIntakeSpeed(IntakeConstants.INTAKE_MOTOR_SPEED);});
            // // Retracts the intake when the retractIntake Button is pressed
            // new JoystickButton(operatorJoystick, Ports.OIOperatorIntakeRetract)
            //     .whenPressed(() ->
            //         {intake.retractIntake();
            //         intake.setIntakeSpeed(0.0);});
        }
        // Set up the transfer 
        if(transferSubsystem.isPresent())
        {
            SK22Transfer transfer = transferSubsystem.get();
        }
        if(visionSubsystem.isPresent())
        {
            SK22Vision vision = visionSubsystem.get();
            acquireTarget.whenPressed(new AcquireTargetCommand(driveSubsystem, vision));
        }
        if(launcherSubsystem.isPresent() && visionSubsystem.isPresent())
        {
            SK22Launcher launcher = launcherSubsystem.get();
            SK22Vision vision = visionSubsystem.get();

            // Shoots ball(s) using the vision system and the launcher
            new JoystickButton(driverLeftJoystick, Ports.OIDriverShoot)
                .whenPressed(new ShootBallsCommand(launcher, vision));
        }
        if(climbSubsystem.isPresent())
        {
            SK22Climb climb = new SK22Climb(
                            new CANSparkMax(Ports.ComplexBrakePivot, ClimbConstants.MOTOR_TYPE),
                            new CANSparkMax(Ports.ComplexRatchetLift, ClimbConstants.MOTOR_TYPE));

            // Extends the climb arms
            new TriggerButton(operatorJoystick, Ports.OIOperatorExtendClimb)
                .whenPressed(climb::extend);
            // Retracts the climb arms
            new JoystickButton(operatorJoystick, Ports.OIOperatorRetractClimb)
                .whenPressed(climb::retract);
            // Goes from one climb rung to the next highest rung
            new JoystickButton(operatorJoystick, Ports.OIOperatorOrchestrateClimb)
                .whenPressed(climb::orchestra);
        }
    }

    private Command makeTrajectoryCommand(Trajectory trajectory, boolean resetOdometry) 
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
        return resetOdometry ? 
            // Run path following command, then stop at the end.
            new SequentialCommandGroup(
                new InstantCommand(() -> driveSubsystem.resetOdometry(trajectory.getInitialPose()), driveSubsystem),
                ramseteCommand.andThen(() -> driveSubsystem.tankDriveVolts(0, 0)))
            :ramseteCommand.andThen(() -> driveSubsystem.tankDriveVolts(0, 0));
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
        // Default Path of Nothign
        autoCommandSelector.setDefaultOption("Do Nothing", AutoCommands.DoNothing);
        
        // Test Paths
        autoCommandSelector.addOption("Drive path from JSON", AutoCommands.DriveSplineFromJSON);
        autoCommandSelector.addOption("Drive canned path", AutoCommands.DriveSplineCanned);
        if(trajectoryCreator.hasTrajectories(new String[]{"1m Forwards", "1m Backwards"}))
        {
            autoCommandSelector.addOption("Drive forwards then backwards 1m", AutoCommands.Drive1mForwardBackward);
        }
       
        // Checking dependencies for autos before giving option to run
        // Adds a majority of autos that have multiple segments
        pathBuilder.displayPossibleAutos((name, command) -> autoCommandSelector.addOption(name, command));
        
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
                    makeTrajectoryCommand(trajectoryCreator.getTrajectory("Ball 1 to Ball 2"), true),
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
