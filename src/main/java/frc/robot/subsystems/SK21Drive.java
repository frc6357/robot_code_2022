//
// SK21Drive: Drivetrain subsystem for the Team 6357 2021 robot
//

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Ports;
import frc.robot.utils.MotorEncoder;

/**
 * The SK21Drive class is the subsystem that controls the drive train of the Robot.
 */
public class SK21Drive extends SKSubsystemBase implements AutoCloseable
{
    private final WPI_TalonFX leftLeader = new WPI_TalonFX(Ports.frontLeftDrive);
    private final WPI_TalonFX leftFollower = new WPI_TalonFX(Ports.backLeftDrive);
    private final MotorEncoder leftMotorEncoder =
            new MotorEncoder(leftLeader, Constants.DriveConstants.ENCODER_DISTANCE_PER_PULSE,
                Constants.DriveConstants.LEFT_ENCODER_REVERSED);
    private final MotorControllerGroup leftGroup =
            new MotorControllerGroup(leftLeader, leftFollower);

    private final WPI_TalonFX rightLeader = new WPI_TalonFX(Ports.frontRightDrive);
    private final WPI_TalonFX rightFollower = new WPI_TalonFX(Ports.backRightDrive);
    private final MotorEncoder rightMotorEncoder =
            new MotorEncoder(rightLeader, Constants.DriveConstants.ENCODER_DISTANCE_PER_PULSE,
                Constants.DriveConstants.RIGHT_ENCODER_REVERSED);
    private final MotorControllerGroup rightGroup =
            new MotorControllerGroup(rightLeader, rightFollower);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final DifferentialDrive drive;
    private final DifferentialDriveOdometry odometry;

    private SendableChooser<Boolean> testControlChooser = new SendableChooser<Boolean>();

    private NetworkTableEntry leftLeaderEntry;
    private NetworkTableEntry leftFollowerEntry;
    private NetworkTableEntry rightLeaderEntry;
    private NetworkTableEntry rightFollowerEntry;
    private NetworkTableEntry speedControllerGroupLeftEntry;
    private NetworkTableEntry speedControllerGroupRightEntry;

    // TODO: The ADIS16448_IMU class is now in WPILib and some of its methods have changed.
    // I've replaced getRotation2d() with Rotation2d.fromDegrees(getAngle()) but we need to check that this returns
    // the expected angle. Spec states that getAngle() is CCW positive for the ADIS16448 class which
    // matches the getRotation2d() method in the old class. Unfortunately, it doesn't match the
    // spec for the basic Gyro classes getAngle() method which is CW positive so there may still be
    // some confusion. According to the DifferentialDriveOdometry documentation, angles need to be
    // provided as CCW positive (see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html)

    /** Creates a new SK21Drive. */
    public SK21Drive()
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
        drive.arcadeDrive(fwd, rot);
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
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading()
    {
        return -gyro.getAngle();
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
    public void close() throws Exception {
        drive.close();
        leftGroup.close();
        rightGroup.close();
        gyro.close();
    }
}
