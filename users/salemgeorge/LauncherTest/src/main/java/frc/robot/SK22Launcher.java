package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.BangBangController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Launcher;

public class SK22Launcher extends SubsystemBase
{
  private final WPI_TalonFX ballLauncher1 = new WPI_TalonFX(Constants.LauncherConstants.ballLauncher1);
  // private final WPI_TalonFX ballLauncher2 = new WPI_TalonFX(Constants.LauncherConstants.ballLauncher2);

  // TODO: When second motor is added, make sure to add ballLauncher2 to the launcherGroup
  private final MotorControllerGroup launcherGroup = new MotorControllerGroup(ballLauncher1);
  
  private final MotorEncoder motorEncoder1 = new MotorEncoder(ballLauncher1, 
                                                              Constants.DriveConstants.ENCODER_DISTANCE_PER_PULSE, 
                                                              Constants.DriveConstants.LEFT_ENCODER_REVERSED);

  // private final MotorEncoder motorEncoder2 = new MotorEncoder(ballLauncher2, 
  //                                                                Constants.DriveConstants.ENCODER_DISTANCE_PER_PULSE, 
  //                                                                Constants.DriveConstants.LEFT_ENCODER_REVERSED);

  private final Launcher launcher;

  /** Creates a new ExampleSubsystem. */
  public SK22Launcher()
  {
    ballLauncher1.setNeutralMode(NeutralMode.Coast);
    ballLauncher1.setInverted(true);
    // ballLauncher2.setNeutralMode(NeutralMode.Coast);
    // ballLauncher2.setInverted(false);
    
    // The 10 is a place holder value for the gear ratio of the motors
    launcher = new Launcher(launcherGroup, motorEncoder1, 10);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    launcher.update();
  }

  @Override
  public void simulationPeriodic()
  {
    // This method will be called once per scheduler run during simulation
  }

  public void enableLauncher() 
  {
    launcher.enableLauncher();
  }

  public void disableLauncher() 
  {
    launcher.disableLauncher();
  }

  public void setLauncherRPM(double rpm) 
  {
    launcher.setTargetMotorRPM(rpm);
  }
}