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
import edu.wpi.first.math.trajectory.Trajectory;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.AutoTools.AutoPaths;
import frc.robot.AutoTools.SK22CommandBuilder;
import frc.robot.AutoTools.TrajectoryBuilder;
import frc.robot.AutoTools.SK22Paths.RunJson;
import frc.robot.commands.AcquireTargetCommand;
import frc.robot.commands.DefaultArcadeDriveCommand;
import frc.robot.commands.DefaultTankDriveCommand;
import frc.robot.commands.EjectBallCommand;
import frc.robot.commands.LoadBallVerticalCommand;
import frc.robot.commands.SetIntakePositionCommand;
import frc.robot.commands.ShootBallsCommand;
import frc.robot.subsystems.SK22Climb;
import frc.robot.subsystems.SK22Drive;
import frc.robot.subsystems.SK22Intake;
import frc.robot.subsystems.SK22Launcher;
import frc.robot.subsystems.SK22Transfer;
import frc.robot.subsystems.SK22Vision;
import frc.robot.subsystems.base.TriggerButton;
import frc.robot.subsystems.base.SuperClasses.Gear;
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
    private UsbCamera camera1;
    private UsbCamera camera2;
    NetworkTableEntry cameraSelection;

    private final TrajectoryBuilder    segmentCreator      =
            new TrajectoryBuilder(Constants.SPLINE_DIRECTORY);
    private final SK22CommandBuilder   pathBuilder         =
            new SK22CommandBuilder(Constants.AUTOS_FOLDER_DIRECTORY, segmentCreator);
    private SendableChooser<AutoPaths> autoCommandSelector = new SendableChooser<AutoPaths>();
    private SendableChooser<Command>   driveModeSelector   = new SendableChooser<Command>();

    private SendableChooser<Trajectory> splineCommandSelector = new SendableChooser<Trajectory>();

    // The robot's subsystems are defined here...
    // TODO: Find which one is high gear and which one is low gear
    private final SK22Drive driveSubsystem = new SK22Drive(new DoubleSolenoid(Ports.BASE_PCM,
        Ports.PNEUMATICS_MODULE_TYPE, Ports.GEAR_SHIFT_HIGH, Ports.GEAR_SHIFT_LOW));
    // These are currently empty and only created in the contructor
    // based on the Subsystem.json file
    private Optional<SK22Intake>      intakeSubsystem   = Optional.empty();
    private Optional<SK22Launcher>    launcherSubsystem = Optional.empty();
    private Optional<SK22Transfer>    transferSubsystem = Optional.empty();
    private final Optional<SK22Climb> climbSubsystem;
    private Optional<SK22Vision>      visionSubsystem   = Optional.empty();

    // Robot External Controllers (Joysticks and Logitech Controller)
    private final FilteredJoystick driverLeftJoystick  =
            new FilteredJoystick(Ports.OI_DRIVER_LEFT_JOYSTICK);
    private final FilteredJoystick driverRightJoystick =
            new FilteredJoystick(Ports.OI_DRIVER_RIGHT_JOYSTICK);
    private final Joystick         operatorJoystick    = new Joystick(Ports.OI_OPERATOR_CONTROLLER);

    // Joystick buttons

    // Note: If we want to continue allowing the choice of both tank drive and arcade drive, we can't use
    // any buttons on the driverRightJoystick since this may not actually be present if arcade drive is
    // chosen!!
    private final JoystickButton driveAcquireTargetBtn =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_ACQUIRE_TARGET);
    private final JoystickButton driveSlowBtn          =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_ACQUIRE_TARGET);
    private final JoystickButton driveLowGearBtn       =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_ACQUIRE_TARGET);
    private final JoystickButton driveHighGearBtn      =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_ACQUIRE_TARGET);
    private final JoystickButton driveShootBtn         =
            new JoystickButton(driverLeftJoystick, Ports.OI_DRIVER_SHOOT);
    private final JoystickButton intakeExtendBtn       =
            new JoystickButton(operatorJoystick, Ports.OI_DRIVER_ACQUIRE_TARGET);
    private final JoystickButton intakeRetractBtn      =
            new JoystickButton(operatorJoystick, Ports.OI_DRIVER_ACQUIRE_TARGET);
    private final JoystickButton transferEjectBallBtn  =
            new JoystickButton(operatorJoystick, Ports.OI_OPERATOR_TRANSFER_EJECT);
    private final JoystickButton transferLoadBallBtn   =
            new JoystickButton(operatorJoystick, Ports.OI_OPERATOR_TRANSFER_LOAD);
    private final TriggerButton  climbExtendBtn        =
            new TriggerButton(operatorJoystick, Ports.OI_OPERATOR_EXTEND_CLIMB);
    private final JoystickButton climbRetractBtn       =
            new JoystickButton(operatorJoystick, Ports.OI_OPERATOR_RETRACT_CLIMB);
    private final JoystickButton climbOrchestrateBtn   =
            new JoystickButton(operatorJoystick, Ports.OI_OPERATOR_ORCHESTRATE_CLIMB);

    private final DefaultArcadeDriveCommand arcadeDrive =
            new DefaultArcadeDriveCommand(driveSubsystem, driverLeftJoystick);
    private final DefaultTankDriveCommand   tankDrive   =
            new DefaultTankDriveCommand(driveSubsystem, driverLeftJoystick, driverRightJoystick);

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

            cameraSelection =
                    NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

            // to change camera displayed feed later on, use the following code
            // cameraSelection.setString(camera2.getName());
            // cameraSelection.setString(camera1.getName());
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
        // TODO: Do we need slow mode now that we have a gear shift? Doesn't low gear achieve
        // the same end?
        driveSlowBtn.whenPressed(() -> driveSubsystem.setMaxOutput(0.5));
        driveSlowBtn.whenReleased(() -> driveSubsystem.setMaxOutput(1));

        // Sets the gear to low when driver clicks setLowGear Buttons
        driveLowGearBtn.whenPressed(() -> driveSubsystem.setGear(Gear.LOW));

        // Sets the gear to high when driver clicks setHighGear Buttons
        driveHighGearBtn.whenPressed(() -> driveSubsystem.setGear(Gear.HIGH));

        // User controls related to the ball intake subsystem
        if (intakeSubsystem.isPresent())
        {
            SK22Intake intake = intakeSubsystem.get();

            // Extends the intake when the extendIntake Button is pressed
            intakeExtendBtn.whenPressed(new SetIntakePositionCommand(intake, true));

            // Retracts the intake when the retractIntake Button is pressed
            intakeRetractBtn.whenPressed(new SetIntakePositionCommand(intake, false));
        }

        // User controls related to the ball transfer subsystem 
        if (transferSubsystem.isPresent())
        {
            SK22Transfer transfer = transferSubsystem.get();

            // Emergency override to eject balls from the horizontal transfer
            transferEjectBallBtn.whenPressed(new EjectBallCommand(transfer, true));
            transferEjectBallBtn.whenReleased(new EjectBallCommand(transfer, false));

            // Emergency override to move ball from the horizontal transfer
            // into the vertical loader.
            transferLoadBallBtn.whenPressed(new LoadBallVerticalCommand(transfer, true));
            transferLoadBallBtn.whenReleased(new LoadBallVerticalCommand(transfer, false));
        }

        if (visionSubsystem.isPresent())
        {
            SK22Vision vision = visionSubsystem.get();
            driveAcquireTargetBtn.whenPressed(new AcquireTargetCommand(driveSubsystem, vision));
            // TODO: How do we break out of this command if it fails to acquire the 
            // target for some reason?
        }

        // User controls related to the ball launcher.
        if (launcherSubsystem.isPresent() && visionSubsystem.isPresent())
        {
            // TODO: I don't think the shoot command should need anything
            // in the vision subsystem. My assumption is that the launcher
            // motor speed will be being set automatically whenever the
            // target is acquired and that the driver will already have 
            // aligned on the target. We should discuss this.
            SK22Launcher launcher = launcherSubsystem.get();
            SK22Vision vision = visionSubsystem.get();

            // Shoots ball(s) using the vision system and the launcher
            driveShootBtn.whenPressed(new ShootBallsCommand(launcher, vision));
        }

        // User controls related to the climbing function.
        if (climbSubsystem.isPresent())
        {
            SK22Climb climb = climbSubsystem.get();

            // TODO: The following are not implemented as commands. If extend and retract
            // are one-off operations, that's probably OK though they may have to be wrapped
            // as instant commands. I imagine that orchestra is a multi-stage, lengthy operation,
            // though so you can't just perform the whole thing in one call since that will
            // completely block the scheduler and the robot will grind to a halt.

            // Extends the climb arms
            climbExtendBtn.whenPressed(climb::extend);

            // Retracts the climb arms
            climbRetractBtn.whenPressed(climb::retract);

            // Goes from one climb rung to the next highest rung
            climbOrchestrateBtn.whenPressed(climb::orchestra);
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
