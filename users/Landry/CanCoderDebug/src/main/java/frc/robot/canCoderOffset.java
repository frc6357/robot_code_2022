package frc.robot;

import com.ctre.phoenix.sensors.*;
/**
 *  Java class that wraps WPI_CANCoder to allow offsetDegrees as parameter that changes 
 * position to an offset position.
 */
public class canCoderOffset extends WPI_CANCoder{

    private double offsetDegrees;

    public canCoderOffset(int deviceNumber, double offsetDegrees)  {
        super(deviceNumber);

        this.offsetDegrees = offsetDegrees;
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.unitString = "deg";
        config.sensorDirection = false; //Counter clockwise positive
        config.magnetOffsetDegrees = this.offsetDegrees;
        config.sensorCoefficient = 360.0 / 4096;

        this.configAllSettings(config);
    }
}
