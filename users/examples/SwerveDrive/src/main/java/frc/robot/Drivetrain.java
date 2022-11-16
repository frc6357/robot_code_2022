// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(Constants.FRONT_LEFT_DRIVING_CAN_ID, 
                                                            Constants.FRONT_LEFT_TURNING_CAN_ID,
                                                            Constants.FRONT_LEFT_CANCODER_ID,
                                                            Constants.FRONT_LEFT_TURNING_ANGLE_OFFSET);
  private final SwerveModule m_frontRight = new SwerveModule(Constants.FRONT_RIGHT_DRIVING_CAN_ID, 
                                                            Constants.FRONT_RIGHT_TURNING_CAN_ID,
                                                            Constants.FRONT_RIGHT_CANCODER_ID,
                                                            Constants.FRONT_RIGHT_TURNING_ANGLE_OFFSET);
  private final SwerveModule m_backLeft = new SwerveModule(Constants.BACK_LEFT_DRIVING_CAN_ID, 
                                                            Constants.BACK_LEFT_TURNING_CAN_ID,
                                                            Constants.BACK_LEFT_CANCODER_ID,
                                                            Constants.BACK_LEFT_TURNING_ANGLE_OFFSET);
  private final SwerveModule m_backRight = new SwerveModule(Constants.BACK_RIGHT_DRIVING_CAN_ID, 
                                                            Constants.BACK_RIGHT_TURNING_CAN_ID,
                                                            Constants.BACK_RIGHT_CANCODER_ID,
                                                            Constants.BACK_RIGHT_TURNING_ANGLE_OFFSET);

  // TODO: wrap ADIS16470_IMU class, and negate getAngle() value to match WIPILIB gyro class

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(m_gyro.getAngle()));

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    double gyroAngle = m_gyro.getAngle();
    SmartDashboard.putNumber("Gyro Angle", gyroAngle);

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyroAngle))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }
}
