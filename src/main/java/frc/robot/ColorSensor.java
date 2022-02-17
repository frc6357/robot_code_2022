
package frc.robot;

import java.lang.Math;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * This class is specifically designed for the 2022 robot therefore many of the
 * functions may not be useful for future designs.
 * This class creates a list of methods that can be used for the 2022 robot's
 * color sensor
 */
public class ColorSensor {

  private final I2C.Port i2c = Ports.i2c;
  /**
   * makes usable colorsensor
   */
  public final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2c);
  Color detectedColor;
  int proximity;
  double maxCount;
  public int count = 1;
  int threshold;

  /**
   * 
   * @param distanceThreshold
   */
  public ColorSensor(int distanceThreshold) 
  {
    threshold = distanceThreshold;
    detectedColor = m_colorSensor.getColor();
    proximity = m_colorSensor.getProximity();
  }

  // Must be called every 20 miliseconds
  // Edit functions called whenever needed
  /**
   * runs the methods constantly
   */
  public void periodic() {
    count++;
    if (count > maxCount) {

      proximity = m_colorSensor.getProximity();
      detectedColor = m_colorSensor.getColor();

      count = 1;
    }
  }

  /**
   * Will return the color that comprises the majority of the input of the
   * colorsensor
   * 
   * @return string value
   */
  public String getColor() {
    // get the specific percentage of red and blue
    double mRed = detectedColor.red;
    double mBlue = detectedColor.blue;

    // will get the distance in IR not in cm
    int distance = proximity;

    if (distance > threshold) {

      if (mRed > mBlue) {
        return "Red";
      } else if (mBlue > mRed) {
        return "Blue";
      }
    }
    return null;
  }

  /**
   * Will return all colors and their percentage of the total inputed colors
   * THIS FUNCTION IS USED FOR TESTING
   */
  public void getAllColors() {
    // get the color from the color sensor
    detectedColor = m_colorSensor.getColor();
    // from the total array of detected colors find red percentage
    double mRed = detectedColor.red;
    // from the total array of detected colors find green percentage
    double mGreen = detectedColor.green;
    // from the total array of detected colors find blue percentage
    double mBlue = detectedColor.red;

    // Multiply to make percentage easier to read
    double redPercentage = mRed * 100;
    double greenPercentage = mGreen * 100;
    double bluePercentage = mBlue * 100;

    // Print to see the total color
    System.out.println("Red " + redPercentage + "%");
    System.out.println("Green " + greenPercentage + "%");
    System.out.println("Blue " + bluePercentage + "%");
  }

  /**
   * Gets the total distance value the ball is from the sensor
   * 
   * @return the distance is cm
   */
  public double getDistance() {
    double cmdistance = proximityToCmConverter();
    return cmdistance;
  }

  /**
   * Will detect if a ball is in range of a certain amount of distance (default
   * 2-5 cm)
   * 
   * @return true if ball is precient or false if not there
   */
  public boolean getBallPresence() {
    int distance = proximity;

    if (distance >= Constants.ColorSensor.MINIMUM_PRSENCE_DISTANCE
        && distance <= Constants.ColorSensor.MAXIMUM_PRESENCE_DISTANCE) {
      return true;
    }
    return false;
  }

  /**
   * Converts the proximity IR values into cm
   * 
   * @return double cm
   */
  public double proximityToCmConverter() {
    double numerator = proximity / 1000;
    double cm = Math.log(numerator) / Math.log(0.64889);
    return cm;
  }

}