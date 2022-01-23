package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;

public class BallSorter {

    private final I2C.Port i2c = I2C.Port.kOnboard;
    public final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2c);
    //Color detectedColor = m_colorSensor.getColor();
    // double IR = m_colorSensor.getIR();
    // int proximity = m_colorSensor.getProximity();


    public BallSorter(int distanceThreshold){
        double m_Distance = distanceThreshold;
    }


    public void colorSensorLogic(double mRed , double mBlue, int distance){
        double redPercentage = mRed * 100;
        double bluePercentage = mBlue * 100;
    
        if(distance > 170){
          if(mRed >  mBlue){
            System.out.println("Red Dominant " + redPercentage + "%");
          }
          else if(mBlue > mRed){
            System.out.println("Blue Dominant " + bluePercentage + "%");
          }
          else{
            System.out.println("None");
          }
        }
      }

}