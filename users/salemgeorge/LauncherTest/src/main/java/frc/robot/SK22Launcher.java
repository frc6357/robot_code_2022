package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Launcher;

public class SK22Launcher extends SubsystemBase
{
  private final WPI_TalonFX ballLauncher1 = new WPI_TalonFX(Constants.LauncherConstants.ballLauncher1);
  private final WPI_TalonFX ballLauncher2 = new WPI_TalonFX(Constants.LauncherConstants.ballLauncher2);

  private final MotorControllerGroup launcherGroup = new MotorControllerGroup(ballLauncher1, ballLauncher2);
  
  // private final MotorEncoder motorEncoder = new MotorEncoder(ballLauncher1, 
                                                                //  Constants.DriveConstants.ENCODER_DISTANCE_PER_PULSE, 
                                                                //  Constants.DriveConstants.LEFT_ENCODER_REVERSED);


  private final Launcher launcher;

  /** Creates a new ExampleSubsystem. */
  public SK22Launcher()
  {
    ballLauncher1.setNeutralMode(NeutralMode.Coast);
    ballLauncher2.setNeutralMode(NeutralMode.Coast);
    
    // The 10 is a place holder value for the gear ratio of the motors
    launcher = new Launcher(launcherGroup, 10);
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