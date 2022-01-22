package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.TuningParams;
import frc.robot.utils.MotorEncoder;

public class SK22Drive extends SKSubsystemBase implements AutoCloseable
{
    private final WPI_TalonFX leftLeader = new WPI_TalonFX(Ports.frontLeftDrive);
    private final WPI_TalonFX leftFollower = new WPI_TalonFX(Ports.backLeftDrive);
    private final MotorEncoder leftMotorEncoder =
            new MotorEncoder(leftLeader, Constants.DriveConstants.kEncoderDistancePerPulse,
                Constants.DriveConstants.kLeftEncoderReversed);
    private final MotorControllerGroup leftGroup =
            new MotorControllerGroup(leftLeader, leftFollower);

    private final WPI_TalonFX rightLeader = new WPI_TalonFX(Ports.frontRightDrive);
    private final WPI_TalonFX rightFollower = new WPI_TalonFX(Ports.backRightDrive);
    private final MotorEncoder rightMotorEncoder =
            new MotorEncoder(rightLeader, Constants.DriveConstants.kEncoderDistancePerPulse,
                Constants.DriveConstants.kRightEncoderReversed);
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
        drive.setDeadband(TuningParams.DEADBAND_TURN);
    }

    public void resetEncoders()
    {
        leftMotorEncoder.resetEncoder();
        rightMotorEncoder.resetEncoder();
    }

    public double getHeading()
    {
        return -gyro.getAngle();
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
