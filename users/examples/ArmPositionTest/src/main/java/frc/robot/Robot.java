/**
 * This is a project to test the RotatingArm base class 
 */
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.base.RotatingArm;


public class Robot extends TimedRobot {
  private final CANSparkMax motor = new CANSparkMax(22, MotorType.kBrushless);
  private final RotatingArm arm;
  private final double motorRotationsToDegrees = (360.0 / 5.0); // 5 to 1 gearbox only 
  // private final double kP = (0.0045 * 0.6);
  // private final double kI = ((2 * kP) / 0.02217);
  // private final double kD =  (0.125 * kP) / 0.02217;
  private final double kP = 0.0008;
  private final double kI = 0.0;
  private final double kD =  0.0;
  private final Joystick m_stick = new Joystick(0); 
  private boolean hasBeenPressed;


  public Robot(){
    arm = new RotatingArm(motor, motor.getEncoder(), motorRotationsToDegrees, kP, kI, kD);
  }

  @Override
  public void teleopPeriodic() {
    arm.update();

    
  }
}
