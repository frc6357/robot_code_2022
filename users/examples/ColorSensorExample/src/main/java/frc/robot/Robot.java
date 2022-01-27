
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.ColorSensorV3;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  // public static ShuffleboardTab colorSensorTab;
  // NetworkTableEntry dominantColor, redPercentage, bluePercentage, distance;
  
  ColorSensor cs1 = new ColorSensor(70);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //colorSensorTab = Shuffleboard.getTab("ColorSensor");
    //dominantColor = colorSensorTab.add("Dominant Color", 0,0).withWidget("Boolean Box").getEntry() ;
    //redPercentage = colorSensorTab.add("Red Percentage", 0,1).withWidget("Text View").getEntry();
    //bluePercentage = colorSensorTab.add("Blue Percentage" , 0,2).withWidget("Text View").getEntry();
    //distance = colorSensorTab.add("Distance", 0,3).withWidget("Text View").getEntry();
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    //dominantColor.setBoolean()

    // Color detectedColor = m_colorSensor.getColor();
    // int proximity = m_colorSensor.getProximity();

    

    // for(int i = 0; i < 1; i+= 0.1){
    //   if()
    // }

    // try {
    //   TimeUnit.SECONDS.sleep(1);
    // } catch (InterruptedException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }

    cs1.periodic();
    //System.out.println("test");
    
    

    
    

    //System.out.print("IR: " + IR);
    //System.out.println(" Proximity: " + proximity);
    //System.out.println(" Red: " + detectedColor.red + " Green: " + detectedColor.green + " Blue: " + detectedColor.blue);

    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
