 
package frc.robot;

import java.lang.Math;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * This class is used for a varity of colorsensor related tasks such as getting distance and the dominate color.
 */
public class ColorSensor {

  private final I2C.Port i2c = Ports.i2c;
  public final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2c);
  Color detectedColor;
  int proximity;
  double maxCount;
  public int count = 1;
  int threshold;



    public ColorSensor(int distanceThreshold){
        threshold = distanceThreshold;
        detectedColor = m_colorSensor.getColor();
        proximity = m_colorSensor.getProximity();
    }

    //Must be called every 20 miliseconds
    public void periodic(){
      count++;
      
      proximity = m_colorSensor.getProximity();
      System.out.println(proximity);

      if(count > maxCount){
        getAllColors();
        proximityToCmConverter();
        count = 1;
      }
    }

    /**
     * Will return the color that comprises the majority of the input of the colorsensor
     */
    public String getColor(){
        //get the specific percentage of red and blue
        double mRed = detectedColor.red;
        double mBlue = detectedColor.blue;

        //will get the distance in IR not in cm
        int distance = proximity;
    
        if(distance > threshold){

          if(mRed >  mBlue){
            return "Red";
          }
          else if(mBlue > mRed){
            return "Blue";
          }
        }
        return null;
      }

      /** 
       * Will return all colors and their percentage of the total inputed colors
       * THIS FUNCTION IS USED FOR TESTING
       */ 
      public void getAllColors(){
        //get the color from the color sensor
        detectedColor = m_colorSensor.getColor();
        //from the total array of detected colors find red percentage
        double mRed = detectedColor.red;
        //from the total array of detected colors find green percentage
        double mGreen = detectedColor.green;
        //from the total array of detected colors find blue percentage
        double mBlue = detectedColor.red;

        //Multiply to make percentage easier to read
        double redPercentage = mRed * 100;
        double greenPercentage = mGreen * 100;
        double bluePercentage = mBlue * 100;

        //Print to see the total color
        // System.out.println("Red " + redPercentage + "%");
        // System.out.println("Green " + greenPercentage + "%");
        // System.out.println("Blue " + bluePercentage + "%");
      }

      public double getDistance(){
        double cmdistance = proximityToCmConverter();
        return cmdistance;
      }

      public boolean getBallPresence(){
        int distance = proximity;
        // if(distnace < threshold){
        //   System.out.println("Presence!");
        //   return true;
        // }
        // System.out.println("No presence!");
        // return false;

        if(distance >= 150 && distance <= 500) {
          System.out.println("Presence!");
          return true;
        }
        System.out.println("No presence!");
        return false;
      }

      public double proximityToCmConverter(){
        double numerator = proximity/1000;
        double cm = Math.log(numerator)/Math.log(0.64889);
        // System.out.println(cm);
        return cm;
      }

}