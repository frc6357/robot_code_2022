// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private Joystick m_joystick;
  
  private SK22Launcher launcher;

  @Override
  public void robotInit() {
    m_joystick = new Joystick(Constants.LauncherConstants.controller);
    SmartDashboard.putNumber("Launcher Set Point", 0.0);
    launcher = new SK22Launcher();
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() 
  {
    launcher.periodic();
  }

  @Override
  public void teleopInit()
  {
    launcher.reset();
  }


  @Override
  public void teleopPeriodic() 
  {
    if(m_joystick.getRawButtonPressed(Constants.LauncherConstants.disableLauncherButton))
    {
      launcher.disableLauncher();
    } 
    else if(m_joystick.getRawButtonPressed(Constants.LauncherConstants.enableLauncherButton))
    {
      launcher.enableLauncher();
    }
    else if(m_joystick.getRawButtonPressed(Constants.LauncherConstants.lowSpeedPresetButton))
    {
      launcher.setLauncherRPM(Constants.LauncherConstants.lowSpeedPreset);
    } 
    else if(m_joystick.getRawButtonPressed(Constants.LauncherConstants.midSpeedPresetButton))
    {
      launcher.setLauncherRPM(Constants.LauncherConstants.midSpeedPreset);
    }
    else if(m_joystick.getRawButtonPressed(Constants.LauncherConstants.highSpeedPresetButton))
    {
      launcher.setLauncherRPM(Constants.LauncherConstants.highSpeedPreset);
    }
    else if(m_joystick.getRawButtonPressed(Constants.LauncherConstants.maxSpeedPresetButton)) 
    {
      launcher.setLauncherRPM(Constants.LauncherConstants.maxSpeedPreset);
    }
    else if(m_joystick.getRawButtonPressed(7)) // back button 
    {
      launcher.setLauncherRPM(SmartDashboard.getNumber("Lanucher Set Point", 0.0));
    }
  }
}