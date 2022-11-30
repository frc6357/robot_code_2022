// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class SwerveModule {
  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared
 
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;

  private final WPI_CANCoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(Constants.DRIVING_PID_KP, Constants.DRIVING_PID_KI, Constants.DRIVING_PID_KD);

  private String m_name;

  

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          Constants.TURNING_PID_KP,
          Constants.TURNING_PID_KI,
          Constants.TURNING_PID_KD,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, turning encoder.
   *
   * @param driveMotorCANId CAN id for the drive motor.
   * @param turningMotorCANId CAN id for the turning motor.
   * @param turningEncoderCANId CAN id for the turning encoder
   */
  public SwerveModule(
      int driveMotorCANId,
      int turningMotorCANId,
      int turningEncoderCANId,
      double turningAngleOffset,
      String name) {
    m_driveMotor = new WPI_TalonFX(driveMotorCANId);
    m_turningMotor = new WPI_TalonFX(turningMotorCANId);

    m_turningEncoder = new WPI_CANCoder(turningEncoderCANId);

    m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180, 0);

    m_turningEncoder.setPosition(turningAngleOffset);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-180, 180);

    m_name = name;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity() * Constants.encoderMetersPerSecondScale, new Rotation2d(m_turningEncoder.getAbsolutePosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    double turnAngle = m_turningEncoder.getAbsolutePosition();
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(turnAngle));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getSelectedSensorVelocity() * Constants.encoderMetersPerSecondScale, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition(), state.angle.getDegrees());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);

    SmartDashboard.putNumber(m_name + " driveOutput", driveOutput);
    SmartDashboard.putNumber(m_name + " driveFeedforward", driveFeedforward);
    SmartDashboard.putNumber(m_name + " turnOutput", turnOutput);
    SmartDashboard.putNumber(m_name + " turnFeedforward", turnFeedforward);
    SmartDashboard.putNumber(m_name + " turnAngle", turnAngle);
  }
}
