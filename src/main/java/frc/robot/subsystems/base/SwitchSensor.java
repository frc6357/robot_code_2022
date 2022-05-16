package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Class for any switch-based sensor that allows normally-open and normally-closed
 * switches to be configured such that we always return true on "pressed" and false
 * on "not pressed".
 */
public class SwitchSensor extends DigitalInput
{
    /**
     * Controls whether the switch value should be inverted before it is returned.
     */
    private final boolean inverted;

    /**
     * This creates a sensor that is connected to a specific DigitalInput and sets
     * the polarity of the switch.
     * 
     * @param channel
     *            The DIO channel that the  sensor is using
     * @param inverted
     *            If true, the raw DIO value read is inverted before being returned
     *            from the get() method.
     */
    public SwitchSensor(int channel, boolean inverted)
    {
        super(channel);
        this.inverted = inverted;
    }

    /**
     * This retuns the value of the sensor taking into count the inverted boolean.
     * 
     * @return The value of the sensor
     */
    @Override
    public boolean get()
    {
        boolean returnVal = super.get();
        return inverted ? !returnVal : returnVal;
    }
}
