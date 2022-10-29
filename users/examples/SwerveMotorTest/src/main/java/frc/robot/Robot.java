/**
 * This is a test program used to exercise each of the 8 motors comprising the
 * swerve drivetrain. Note that this IS NOT something that provides a driveable
 * robot!! Use it it purely to test connections and directions with the drivetrain
 * on blocks!
 */

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private Joystick m_DriveController;
  private Joystick m_AngleController;

  private final MotorController m_FrontLeftDriveMotor  = new WPI_TalonFX(Ports.FRONT_LEFT_DRIVE_MOTOR);
  private final MotorController m_FrontRightDriveMotor = new WPI_TalonFX(Ports.FRONT_RIGHT_DRIVE_MOTOR);
  private final MotorController m_BackLeftDriveMotor   = new WPI_TalonFX(Ports.BACK_LEFT_DRIVE_MOTOR);
  private final MotorController m_BackRightDriveMotor  = new WPI_TalonFX(Ports.BACK_RIGHT_DRIVE_MOTOR);

  private final MotorController m_FrontLeftAngleMotor  = new WPI_TalonFX(Ports.FRONT_LEFT_ANGLE_MOTOR);
  private final MotorController m_FrontRightAngleMotor = new WPI_TalonFX(Ports.FRONT_RIGHT_ANGLE_MOTOR);
  private final MotorController m_BackLeftAngleMotor  = new WPI_TalonFX(Ports.BACK_LEFT_ANGLE_MOTOR);
  private final MotorController m_BackRightAngleMotor = new WPI_TalonFX(Ports.BACK_RIGHT_ANGLE_MOTOR);
  
  private final WPI_CANCoder    m_FrontLeftAngleEncoder = new WPI_CANCoder(Ports.FRONT_LEFT_ANGLE_ENCODER);
  private final WPI_CANCoder    m_FrontRightAngleEncoder = new WPI_CANCoder(Ports.FRONT_RIGHT_ANGLE_ENCODER);
  private final WPI_CANCoder    m_BackLeftAngleEncoder = new WPI_CANCoder(Ports.BACK_LEFT_ANGLE_ENCODER);
  private final WPI_CANCoder    m_BackRightAngleEncoder = new WPI_CANCoder(Ports.BACK_RIGHT_ANGLE_ENCODER);

  @Override
  public void robotInit() {
    // TODO: We may need to invert some or all of the motor directions but, for now, we're
    // just setting everything to use the defaults so that we can verify that all the motors
    // are working.

    m_DriveController = new Joystick(Ports.OI_DRIVE_CONTROLLER);
    m_AngleController  = new Joystick(Ports.OI_ANGLE_CONTROLLER);
  }

  @Override
  public void teleopPeriodic() {
    double FLSpeed;
    double FRSpeed;
    double BLSpeed;
    double BRSpeed;

    // Read each of the 4 axes on the drive controller and use the values to
    // directly set the respective motor's speed. This is NOT something we 
    // would do on a "real" robot but is adequate for a simple test.
    FLSpeed = m_DriveController.getRawAxis(Ports.OI_DRIVE_FRONT_LEFT);
    FRSpeed = m_DriveController.getRawAxis(Ports.OI_DRIVE_FRONT_RIGHT);
    BLSpeed = m_DriveController.getRawAxis(Ports.OI_DRIVE_BACK_LEFT);
    BRSpeed = m_DriveController.getRawAxis(Ports.OI_DRIVE_BACK_RIGHT);
    SmartDashboard.putNumber("FL Drive Speed", FLSpeed);
    SmartDashboard.putNumber("FR Drive Speed", FRSpeed);
    SmartDashboard.putNumber("BL Drive Speed", BLSpeed);
    SmartDashboard.putNumber("BR Drive Speed", BRSpeed);
    m_FrontLeftDriveMotor.set(FLSpeed);
    m_FrontRightDriveMotor.set(FRSpeed);
    m_BackLeftDriveMotor.set(BLSpeed);
    m_BackRightDriveMotor.set(BRSpeed);

    FLSpeed = m_AngleController.getRawAxis(Ports.OI_ANGLE_FRONT_LEFT);
    FRSpeed = m_AngleController.getRawAxis(Ports.OI_ANGLE_FRONT_RIGHT);
    BLSpeed = m_AngleController.getRawAxis(Ports.OI_ANGLE_BACK_LEFT);
    BRSpeed = m_AngleController.getRawAxis(Ports.OI_ANGLE_BACK_RIGHT);
    SmartDashboard.putNumber("FL Angle Speed", FLSpeed);
    SmartDashboard.putNumber("FR Angle Speed", FRSpeed);
    SmartDashboard.putNumber("BL Angle Speed", BLSpeed);
    SmartDashboard.putNumber("BR Angle Speed", BRSpeed);
    m_FrontLeftAngleMotor.set(FLSpeed);
    m_FrontRightAngleMotor.set(FRSpeed);
    m_BackLeftAngleMotor.set(BLSpeed);
    m_BackRightAngleMotor.set(BRSpeed);

    // Read back the angle encoders.
    double FLAngle = m_FrontLeftAngleEncoder.getAbsolutePosition();
    double FRAngle = m_FrontRightAngleEncoder.getAbsolutePosition();
    double BLAngle = m_BackLeftAngleEncoder.getAbsolutePosition();
    double BRAngle = m_BackRightAngleEncoder.getAbsolutePosition();
    SmartDashboard.putNumber("FL Angle Encoder", FLAngle);
    SmartDashboard.putNumber("FR Angle Encoder", FRAngle);
    SmartDashboard.putNumber("BL Angle Encoder", BLAngle);
    SmartDashboard.putNumber("BR Angle Encoder", BRAngle);
  }
}
