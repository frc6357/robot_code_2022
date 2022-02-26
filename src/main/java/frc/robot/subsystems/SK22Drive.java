//
// SK22Drive: Drivetrain subsystem for the Team 6357 2022 robot
//
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Ports;
import frc.robot.utils.DifferentialDrivetrain;
import frc.robot.utils.MotorEncoder;

/**
 * The SK22Drive class is the subsystem that controls the drive train of the Robot.
 */
public class SK22Drive extends SKSubsystemBase implements AutoCloseable, DifferentialDrivetrain
{
    private final WPI_TalonFX          leftLeader       = new WPI_TalonFX(Ports.FRONT_LEFT_DRIVE);
    private final WPI_TalonFX          leftFollower     = new WPI_TalonFX(Ports.BACK_LEFT_DRIVE);
    private final MotorEncoder         leftMotorEncoder = new MotorEncoder(leftLeader,
        DriveConstants.ENCODER_DISTANCE_PER_PULSE, DriveConstants.LEFT_ENCODER_REVERSED);
    private final MotorControllerGroup leftGroup        =
            new MotorControllerGroup(leftLeader, leftFollower);

    private final WPI_TalonFX          rightLeader       = new WPI_TalonFX(Ports.FRONT_RIGHT_DRIVE);
    private final WPI_TalonFX          rightFollower     = new WPI_TalonFX(Ports.BACK_RIGHT_DRIVE);
    private final MotorEncoder         rightMotorEncoder = new MotorEncoder(rightLeader,
        DriveConstants.ENCODER_DISTANCE_PER_PULSE, DriveConstants.RIGHT_ENCODER_REVERSED);
    private final MotorControllerGroup rightGroup        =
            new MotorControllerGroup(rightLeader, rightFollower);

    private final ADIS16448_IMU gyro = new ADIS16448_IMU();

    private final DifferentialDrive         drive;
    private final DifferentialDriveOdometry odometry;

    private SendableChooser<Boolean> testControlChooser = new SendableChooser<Boolean>();

    private NetworkTableEntry leftLeaderEntry;
    private NetworkTableEntry leftFollowerEntry;
    private NetworkTableEntry rightLeaderEntry;
    private NetworkTableEntry rightFollowerEntry;
    private NetworkTableEntry speedControllerGroupLeftEntry;
    private NetworkTableEntry speedControllerGroupRightEntry;

    private Field2d field = new Field2d();

    private boolean reversed = false;

    /**
     * Creates a SK22Drive subsystem controlling the drivetrain.
     */
    public SK22Drive()
    {
        resetEncoders();
        gyro.reset();

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(this.getHeading()));
        leftLeader.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);

        // The 2022 version of DifferentialDrive no longer automatically inverts one
        // side of the robot so we need to do this ourselves.
        rightGroup.setInverted(true);

        drive = new DifferentialDrive(leftGroup, rightGroup);
        drive.setDeadband(DriveConstants.DEADBAND_TURN);

        SmartDashboard.putString("Direction", "Intake");
    }
    
    @Override
    public void periodic()
    {
        double leftEncoderDistanceMeters = leftMotorEncoder.getPositionMeters();
        double rightEncoderDistanceMeters = rightMotorEncoder.getPositionMeters();
        double leftEncoderSpeedMeters = leftMotorEncoder.getVelocityMeters();
        double rightEncoderSpeedMeters = rightMotorEncoder.getVelocityMeters();
        // Update the odometry in the periodic block
        odometry.update(Rotation2d.fromDegrees(this.getHeading()), leftEncoderDistanceMeters,
            rightEncoderDistanceMeters);

            SmartDashboard.putNumber("Left Wheel Distance", leftEncoderDistanceMeters);
        SmartDashboard.putNumber("Right Wheel Distance", rightEncoderDistanceMeters);
        SmartDashboard.putNumber("Left Wheel Speed", leftEncoderSpeedMeters);
        SmartDashboard.putNumber("Right Wheel Speed", rightEncoderSpeedMeters);
        SmartDashboard.putNumber("Gyro Angle", this.getHeading());

        field.setRobotPose(odometry.getPoseMeters());
        SmartDashboard.putData("Field", field);
    }
    
    /**
     * Sets whether or not the launcher should be the "forward"
     * of the robot
     * 
     * @param reversed Whether the robot should be set launcher forward
     */
    public void setBackwardsDirection(boolean reversed)
    {
        this.reversed = reversed;
        SmartDashboard.putString("Direction", reversed ? "Launcher" : "Intake");
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose()
    {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(leftMotorEncoder.getVelocityMeters(),
            rightMotorEncoder.getVelocityMeters());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose
     *            The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose)
    {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(gyro.getAngle()));
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd
     *            the commanded forward movement
     * @param rot
     *            the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot)
    {
        if (reversed)
        {
            drive.arcadeDrive(-fwd, rot);
        }
        else
        {
            drive.arcadeDrive(fwd, rot);
        }
    }

    /**
     * Drives the robot using tank controls
     * 
     * @param leftSpeed
     *            the speed of the left motors
     * @param rightSpeed
     *            the speed of the right motors
     */
    public void tankDrive(double leftSpeed, double rightSpeed)
    {
        if (reversed)
        {
            drive.tankDrive(-rightSpeed, -leftSpeed);
        }
        else
        {
            drive.tankDrive(leftSpeed, rightSpeed);
        }
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     * 
     * @param leftVolts
     *            the commanded left output
     * @param rightVolts
     *            the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts)
    {
        leftGroup.setVoltage(leftVolts);
        rightGroup.setVoltage(rightVolts);
        drive.feed();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders()
    {
        leftMotorEncoder.resetEncoder();
        rightMotorEncoder.resetEncoder();
    }

    /** Reset the ADS gyro */
    public void resetGyro()
    {
        gyro.reset();
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance()
    {
        // return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
        return (leftMotorEncoder.getPositionMeters() + rightMotorEncoder.getPositionMeters()) / 2.0;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput
     *            the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput)
    {
        drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading()
    {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return yaw axis angle in degrees (CCW positive)
     */
    public double getHeading()
    {
        return gyro.getAngle();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate()
    {
        return -gyro.getRate();
    }

    @Override
    public void initializeTestMode()
    {
        testControlChooser.setDefaultOption("By left/right group", true);
        testControlChooser.addOption("Individual motors", false);

        Shuffleboard.getTab("Drive").add("Control Mode", testControlChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1).withPosition(0, 0);

        leftLeaderEntry = Shuffleboard.getTab("Drive").add("leftLeader", 1)
            .withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1).withPosition(0, 1).getEntry();

        leftFollowerEntry = Shuffleboard.getTab("Drive").add("leftFollower", 1)
            .withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1).withPosition(2, 1).getEntry();

        rightLeaderEntry = Shuffleboard.getTab("Drive").add("rightLeader", 1)
            .withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1).withPosition(0, 2).getEntry();

        rightFollowerEntry = Shuffleboard.getTab("Drive").add("rightFollower", 1)
            .withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1).withPosition(2, 2).getEntry();

        speedControllerGroupLeftEntry = Shuffleboard.getTab("Drive")
            .add("SpeedControllerGroupLeft", 1).withWidget(BuiltInWidgets.kNumberSlider)
            .withSize(2, 1).withPosition(0, 3).getEntry();

        speedControllerGroupRightEntry = Shuffleboard.getTab("Drive")
            .add("SpeedControllerGroupRight", 1).withWidget(BuiltInWidgets.kNumberSlider)
            .withSize(2, 1).withPosition(2, 3).getEntry();
    }

    /**
     * Creates a command using a trajectory.
     * 
     * @param trajectory
     *            The {@link Trajectory} to be made into a command
     * @param resetOdometry
     *            Whether the command should reset the odometry before starting the path
     * @return The command that uses {@link SK22Drive} to run a path.
     */
    public Command makeTrajectoryCommand(Trajectory trajectory, boolean resetOdometry)
    {
        RamseteCommand ramseteCommand = new RamseteCommand(trajectory, this::getPose,
            AutoConstants.RAMSETE_CONTROLLER, AutoConstants.SIMPLE_MOTOR_FEEDFORWARD,
            DriveConstants.DRIVE_KINEMATICS, this::getWheelSpeeds, AutoConstants.PID_CONTROLLER_LEFT,
            AutoConstants.PID_CONTROLLER_RIGHT,
            // RamseteCommand passes volts to the callback
            this::tankDriveVolts, this);

        // Tell the robot where it is starting from if this is the first trajectory of a path.
        return resetOdometry
            ?
            // Run path following command, then stop at the end.
            new SequentialCommandGroup(
                new InstantCommand(() -> this.resetOdometry(trajectory.getInitialPose()), this),
                ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0)))
            : ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
    }

    @Override
    public void testModePeriodic()
    {
        if (testControlChooser.getSelected())
        {
            leftGroup.set(speedControllerGroupLeftEntry.getValue().getDouble());
            rightGroup.set(speedControllerGroupRightEntry.getValue().getDouble());
        }
        else
        {
            leftLeader.set(leftLeaderEntry.getValue().getDouble());
            leftFollower.set(leftFollowerEntry.getValue().getDouble());
            rightLeader.set(rightLeaderEntry.getValue().getDouble());
            rightFollower.set(rightFollowerEntry.getValue().getDouble());
        }

    }

    @Override
    public void enterTestMode()
    {
        speedControllerGroupRightEntry.setDouble(0.0);
        speedControllerGroupLeftEntry.setDouble(0.0);
        leftFollowerEntry.setDouble(0.0);
        rightFollowerEntry.setDouble(0.0);
        rightLeaderEntry.setDouble(0.0);
        leftLeaderEntry.setDouble(0.0);

    }

    // The close() method is required to allow use with jUnit unit tests (see src/test/java).
    // To use this, the class must implement the AutoCloseable interface.
    // This is called to clean up between each test and must close all downstream objects
    // before returning.
    @Override
    public void close() throws Exception
    {
        drive.close();
        leftGroup.close();
        rightGroup.close();
        gyro.close();
    }
}
