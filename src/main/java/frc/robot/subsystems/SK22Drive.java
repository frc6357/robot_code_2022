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
import frc.robot.Constants.DriveConstants;
import frc.robot.Ports;
import frc.robot.utils.MotorEncoder;

public class SK22Drive extends SKSubsystemBase implements AutoCloseable
{
    private final WPI_TalonFX leftLeader = new WPI_TalonFX(Ports.frontLeftDrive);
    private final WPI_TalonFX leftFollower = new WPI_TalonFX(Ports.backLeftDrive);
    private final MotorEncoder leftMotorEncoder =
            new MotorEncoder(leftLeader, DriveConstants.ENCODER_DISTANCE_PER_PULSE,
                DriveConstants.LEFT_ENCODER_REVERSED);
    private final MotorControllerGroup leftGroup =
            new MotorControllerGroup(leftLeader, leftFollower);

    private final WPI_TalonFX rightLeader = new WPI_TalonFX(Ports.frontRightDrive);
    private final WPI_TalonFX rightFollower = new WPI_TalonFX(Ports.backRightDrive);
    private final MotorEncoder rightMotorEncoder =
            new MotorEncoder(rightLeader, DriveConstants.ENCODER_DISTANCE_PER_PULSE,
                DriveConstants.RIGHT_ENCODER_REVERSED);
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

    public Pose2d getPose()
    {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(leftMotorEncoder.getVelocityMeters(),
            rightMotorEncoder.getVelocityMeters());
    }

    public void resetOdometry(Pose2d pose)
    {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(gyro.getAngle()));
    }

    public void arcadeDrive(double fwd, double rot)
    {
        drive.arcadeDrive(fwd, rot);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts)
    {
        leftGroup.setVoltage(leftVolts);
        rightGroup.setVoltage(rightVolts);
        drive.feed();
    }

    public void resetEncoders()
    {
        leftMotorEncoder.resetEncoder();
        rightMotorEncoder.resetEncoder();
    }

    public void resetGyro()
    {
        gyro.reset();
    }

    public double getAverageEncoderDistance()
    {
        // return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
        return (leftMotorEncoder.getPositionMeters() + rightMotorEncoder.getPositionMeters()) / 2.0;
    }

    public void setMaxOutput(double maxOutput)
    {
        drive.setMaxOutput(maxOutput);
    }

    public void zeroHeading()
    {
        gyro.reset();
    }

    public double getHeading()
    {
        return -gyro.getAngle();
    }

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

    @Override
    public void close() throws Exception {
        drive.close();
        leftGroup.close();
        rightGroup.close();
        gyro.close();
    }
}
