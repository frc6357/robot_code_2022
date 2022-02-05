 /**
   * This class is used to 
   * 
   * 
   */
package frc.robot;

import java.lang.Math;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor {

  private final I2C.Port i2c = Ports.i2c;
  public final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2c);
  Color detectedColor;
  int proximity;
  double maxCount;
  public int count = 1;
    //Color detectedColor = m_colorSensor.getColor();
    // double IR = m_colorSensor.getIR();
    // int proximity = m_colorSensor.getProximity();


    public ColorSensor(int distanceThreshold){
        int m_Distance = distanceThreshold;
        detectedColor = m_colorSensor.getColor();
        proximity = m_colorSensor.getProximity();
    }

    //Must be called every 20 miliseconds
    public void periodic(){
      count++;
      

      if(count > maxCount){
        getAllColors();
        irToCentimeterConverter();
        count = 1;
      }
    }
    public void getColor(){
        double mRed = detectedColor.red;
        double mBlue = detectedColor.red;
        int distance = proximity;
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

      public void getAllColors(){
        detectedColor = m_colorSensor.getColor();
        double mRed = detectedColor.red;
        double mGreen = detectedColor.green;
        double mBlue = detectedColor.red;

        double redPercentage = mRed * 100;
        double greenPercentage = mGreen * 100;
        double bluePercentage = mBlue * 100;

        System.out.println("Red " + redPercentage + "%");
        System.out.println("Green " + greenPercentage + "%");
        System.out.println("Blue " + bluePercentage + "%");
      }

      public void getDistance(){

      }

      public double irToCentimeterConverter(){
        double numerator = proximity/1000;
        double cm = Math.log(numerator)/Math.log(0.64889);
        System.out.println(cm);
        return cm;
      }

}