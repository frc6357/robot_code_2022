/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.Set;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LauncherConstants;
import frc.robot.AutoTools.AutoPaths;
import frc.robot.AutoTools.SK22CommandBuilder;
import frc.robot.AutoTools.TrajectoryBuilder;
import frc.robot.AutoTools.SK22Paths.Drive1mForwardBackward;
import frc.robot.AutoTools.SK22Paths.RunJson;
import frc.robot.commands.AcquireTargetCommand;
import frc.robot.commands.DefaultArcadeDriveCommand;
import frc.robot.commands.DefaultTankDriveCommand;
import frc.robot.commands.EjectBallCommand;
import frc.robot.commands.LoadBallVerticalCommand;
import frc.robot.commands.ReverseVerticalTransferCommand;
import frc.robot.commands.SetIntakePositionCommand;
import frc.robot.commands.ShootBallsCommand;
import frc.robot.subsystems.SK22Drive;
import frc.robot.subsystems.SK22Intake;
import frc.robot.subsystems.SK22Launcher;
import frc.robot.subsystems.SK22SimpleClimb;
import frc.robot.subsystems.SK22Transfer;
import frc.robot.subsystems.SK22Vision;
import frc.robot.subsystems.base.Dpad;
import frc.robot.subsystems.base.DpadDownButton;
import frc.robot.subsystems.base.DpadUpButton;
import frc.robot.subsystems.base.TriggerButton;
import frc.robot.utils.FilteredJoystick;
import frc.robot.utils.SubsystemControls;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the
 * {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure
 * of the robot (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer
{
    /**
     * The USB Camera for the Robot.
     */
    private UsbCamera         camera1;
    private UsbCamera         camera2;
    private VideoSink         server;

    private final SK22CommandBuilder pathBuilder;
    private final TrajectoryBuilder    segmentCreator      =
            new TrajectoryBuilder(Constants.SPLINE_DIRECTORY);
    private SendableChooser<AutoPaths> autoCommandSelector = new SendableChooser<AutoPaths>();
    private SendableChooser<Command>   driveModeSelector   = new SendableChooser<Command>();

    private SendableChooser<Trajectory> splineCommandSelector = new SendableChooser<Trajectory>();

    // The robot's subsystems are defined here...

    // Drivetrain is the only subsystem that is not optional.
    private final SK22Drive driveSubsystem = new SK22Drive();

    // These are currently empty and only created in the constructor
    // based on the Subsystem.json file
    private Optional<SK22Intake>       intakeSubsystem       = Optional.empty();
    private Optional<SK22Launcher>     launcherSubsystem     = Optional.empty();
    private Optional<SK22Transfer>     transferSubsystem     = Optional.empty();
    private Optional<SK22SimpleClimb>  simpleClimbSubsystem  = Optional.empty();
    private Optional<SK22Vision>       visionSubsystem       = Optional.empty();

    // Robot External Controllers (Joysticks and Logitech Controller)
    private final FilteredJoystick driverLeftJoystick  =
            new FilteredJoystick(Ports.OI_DRIVER_LEFT_JOYSTICK);
    private final FilteredJoystick driverRightJoystick =
            new FilteredJoystick(Ports.OI_DRIVER_RIGHT_JOYSTICK);
    private final Joystick         operatorJoystick    = new Joystick(Ports.OI_OPERATOR_CONTROLLER);

    private final PneumaticHub pneumaticHubIntake = new PneumaticHub(2);
    // private final Compressor phCompressor = new Compressor(2, PneumaticsModuleType.REVPH);

    // Joystick buttons

    // Note: If we want to continue allowing the choice of both tank drive and arcade drive, we can't use
    // any buttons on the driverRightJoystick since this may not actually be present if arcade drive is
    // chosen!!

    // Verified according to 2022 Controller Mapping document on 2/12/2022 
    private final JoystickButton driveAcquireTargetBtn =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_ACQUIRE_TARGET);
    private final JoystickButton driveSlowBtn          =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_SLOWMODE);
    private final JoystickButton driveShootBtn         =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_SHOOT);
    private final JoystickButton driverLauncherLowBtn   =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_LAUNCHER_LOW);
    private final JoystickButton driverLauncherMaxBtn   =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_LAUNCHER_MAX);
    private final JoystickButton driverLauncherOffBtn  =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_LAUNCHER_OFF);
    private final JoystickButton driverEnableVision    =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_VISION_ON);    
    private final JoystickButton driverDisableVision    =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_VISION_OFF); 
    private final Dpad           dpad                  =
            new Dpad(driverLeftJoystick, Ports.OI_DRIVER_REVERSE);
    private final DpadDownButton reverseOnBtn          = new DpadDownButton(dpad);
    private final DpadUpButton   reverseOffBtn         = new DpadUpButton(dpad);
    private final JoystickButton intakeExtendBtn       =
            new JoystickButton(operatorJoystick, Ports.OI_OPERATOR_INTAKE_EXTEND);
    private final JoystickButton intakeRetractBtn      =
            new JoystickButton(operatorJoystick, Ports.OI_OPERATOR_INTAKE_RETRACT);
    private final JoystickButton transferEjectBallBtn  =
            new JoystickButton(operatorJoystick, Ports.OI_OPERATOR_TRANSFER_EJECT);
    private final JoystickButton transferLoadBallBtn   =
            new JoystickButton(operatorJoystick, Ports.OI_OPERATOR_TRANSFER_LOAD);
    private final JoystickButton reverseVerticalTransferBtn =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_VERTICAL_TRANSFER_REVERSE);
            
    private final TriggerButton  climbExtendBtn        =
            new TriggerButton(operatorJoystick, Ports.OI_OPERATOR_EXTEND_CLIMB);
    private final JoystickButton climbRetractBtn       =
            new JoystickButton(operatorJoystick, Ports.OI_OPERATOR_RETRACT_CLIMB);

    private final DefaultArcadeDriveCommand arcadeDrive =
            new DefaultArcadeDriveCommand(driveSubsystem, driverLeftJoystick);
    private final DefaultTankDriveCommand   tankDrive   =
            new DefaultTankDriveCommand(driveSubsystem, driverLeftJoystick, driverRightJoystick);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        // phCompressor.enableAnalog(0, 120);
        pneumaticHubIntake.enableCompressorDigital();
        
        File deployDirectory = Filesystem.getDeployDirectory();

        ObjectMapper mapper = new ObjectMapper();
        JsonFactory factory = new JsonFactory();
 
        try
        {
            // Looking for the Subsystems.json file in the deploy directory
            JsonParser parser =
                    factory.createParser(new File(deployDirectory, Constants.SUBSYSTEM));
            SubsystemControls subsystems = mapper.readValue(parser, SubsystemControls.class);

            // Instantiating subsystems if they are present
            // This is decided by looking at Subsystems.json
            if (subsystems.isIntakePresent())
            {
                intakeSubsystem = Optional.of(new SK22Intake());
            }
            if (subsystems.isLauncherPresent())
            {
                launcherSubsystem = Optional.of(new SK22Launcher());
            }
            if (subsystems.isTransferPresent())
            {
                transferSubsystem = Optional.of(new SK22Transfer());
            }
            if (subsystems.isVisionPresent())
            {
                visionSubsystem = Optional.of(new SK22Vision());
            }
            if (subsystems.isSimpleClimbPresent())
            {
                simpleClimbSubsystem = Optional.of(new SK22SimpleClimb());
            }
        }
        catch (IOException e)
        {
            DriverStation.reportError("Failure to read Subsystem Control File!", e.getStackTrace());
        }

        pathBuilder =
                new SK22CommandBuilder(Constants.AUTOS_FOLDER_DIRECTORY, segmentCreator,
                    intakeSubsystem, transferSubsystem, launcherSubsystem);

        // Configure the button bindings
        configureButtonBindings();

        configureShuffleboard();

        resetDriveDefaultCommand();

        // Driver camera configuration
        if (RobotBase.isReal())
        {
            camera1 = CameraServer.startAutomaticCapture("Driver Front Camera", 0);
            camera1.setResolution(240, 240);
            camera1.setFPS(15);

            camera2 = CameraServer.startAutomaticCapture("Driver Rear Camera", 1);
            camera2.setResolution(240, 240);
            camera2.setFPS(15);

            server = CameraServer.getServer();

            camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        }
    }

    /**
     * Used to reset the drive subsystem's default command to the one decided by the user
     * through a Sendable Chooser
     */
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
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Turns on slowmode when driver presses slowmode button, giving more manueverability.
        driveSlowBtn.whenPressed(() -> driveSubsystem.setMaxOutput(0.5));
        driveSlowBtn.whenReleased(() -> driveSubsystem.setMaxOutput(1));

        // Sets both the direction controls and the camera selection
        reverseOffBtn.whenPressed(() -> driveSubsystem.setBackwardsDirection(false))
            .whenPressed(() -> server.setSource(camera1));
        reverseOnBtn.whenPressed(() -> driveSubsystem.setBackwardsDirection(true))
            .whenPressed(() -> server.setSource(camera2));

        // User controls related to the ball intake subsystem
        if (intakeSubsystem.isPresent() && transferSubsystem.isPresent())
        {
            SK22Intake intake = intakeSubsystem.get();
            SK22Transfer transfer = transferSubsystem.get();

            // Extends the intake when the extendIntake Button is pressed
            intakeExtendBtn.whenPressed(new SetIntakePositionCommand(intake, transfer, true));

            // Retracts the intake when the retractIntake Button is pressed
            intakeRetractBtn.whenPressed(new SetIntakePositionCommand(intake, transfer, false));
        }

        // User controls related to the ball transfer subsystem 
        if (transferSubsystem.isPresent())
        {
            SK22Transfer transfer = transferSubsystem.get();

            // Move ball through the horizontal transfer to the base of the vertical shaft.
            transferLoadBallBtn.whenHeld(new LoadBallVerticalCommand(transfer), true);
        }

        if (visionSubsystem.isPresent())
        {
            SK22Vision vision = visionSubsystem.get();

            PowerDistribution phHub = new PowerDistribution(1, ModuleType.kRev);

            driveAcquireTargetBtn
                .whenHeld(new AcquireTargetCommand(driveSubsystem, vision), true);

            driverEnableVision
                .whenPressed(() -> phHub.setSwitchableChannel(true));
            driverDisableVision
                .whenPressed(() -> phHub.setSwitchableChannel(false));
        }

        // User controls related to the ball launcher and transfer related things.
        if (launcherSubsystem.isPresent() && transferSubsystem.isPresent())
        {
            SK22Launcher launcher = launcherSubsystem.get();
            SK22Transfer transfer = transferSubsystem.get();

            // Shoots ball(s) using the launcher
            driveShootBtn.whenHeld(new ShootBallsCommand(launcher, transfer), true);
            launcher.setLauncherRPM(0.0);
            launcher.enableLauncher();

            // Buttons to turn off the launcher flywheel
            driverLauncherOffBtn.whenPressed(() -> launcher.setLauncherRPM(0.0));

            // Buttons to turn on the launcher to preset rpms
            driverLauncherLowBtn.whenPressed(
                () -> launcher.setLauncherRPM(LauncherConstants.LOW_SPEED_PRESET));
            driverLauncherMaxBtn.whenPressed(
                () -> launcher.setLauncherRPM(LauncherConstants.MAX_SPEED_PRESET));

            reverseVerticalTransferBtn.whenHeld(new ReverseVerticalTransferCommand(launcher), true);

            // Emergency override to eject balls from the horizontal transfer
            transferEjectBallBtn.whenHeld(new EjectBallCommand(transfer, launcher), true);
        }

        // User controls related to the climbing function.
        if (simpleClimbSubsystem.isPresent())
        {
            SK22SimpleClimb simpleClimb = simpleClimbSubsystem.get();

            // Extends the climb arm
            climbExtendBtn.whenPressed(() -> simpleClimb.raiseSimpleArm());

            // Retracts the climb arm
            climbRetractBtn.whenPressed(() -> simpleClimb.lowerSimpleArm());
        }
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
        autoCommandSelector.addOption("1m forwards backwards", new Drive1mForwardBackward());

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
