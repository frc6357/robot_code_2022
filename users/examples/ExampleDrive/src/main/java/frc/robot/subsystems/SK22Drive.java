// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class SK22Drive extends SubsystemBase {

  private final WPI_TalonFX leftLeader = new WPI_TalonFX(Ports.frontLeftDrive);
  private final WPI_TalonFX leftFollower = new WPI_TalonFX(Ports.backLeftDrive);
   
  private final SpeedControllerGroup leftGroup =
          new SpeedControllerGroup(leftLeader, leftFollower);

  private final WPI_TalonFX rightLeader = new WPI_TalonFX(Ports.frontRightDrive);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(Ports.backRightDrive);
    
  private final SpeedControllerGroup rightGroup =
          new SpeedControllerGroup(rightLeader, rightFollower);

  /** Creates a new ExampleSubsystem. */
  public SK22Drive() {
    leftLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setSpeed(double leftSpeed, double rightSpeed){
      leftGroup.set(leftSpeed);
      rightGroup.set(rightSpeed);
  }
}
