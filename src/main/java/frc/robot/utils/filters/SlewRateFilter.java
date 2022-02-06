package frc.robot.utils.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class SlewRateFilter implements Filter
{

    private final SlewRateLimiter slewFilter;
    private double reversed = 1.0;

    /**
     * Creates a filter with a max specified change rate. This prevents an input from
     * changing too quickly.
     * 
     * @param filterRate
     *            The max rate at which the input can change
     */
    public SlewRateFilter(double filterRate)
    {
        slewFilter = new SlewRateLimiter(filterRate);
    }

    /**
     * Creates a filter with a max specified change rate. This prevents an input from
     * changing too quickly.
     * 
     * @param filterRate
     *            The max rate at which the input can change
     * @param reversed
     *            Whether the sign of the input should be inverted
     */
    public SlewRateFilter(double filterRate, boolean reversed)
    {
        slewFilter = new SlewRateLimiter(filterRate);
        this.reversed = reversed ? -1.0 : 1.0;
    }

    /**
     * Filters the given input value per the rules of this Filter, and returns the
     * filtered value.
     * 
     * @param rawAxis
     *            The actual value being returned by the raw data
     * @return The filtered data to be passed to the motor
     */
    public double filter(double rawAxis)
    {
        return reversed * slewFilter.calculate(rawAxis);
    }

    /**
     * Sets whether the sign of input should be inverted
     * 
     * @param reversed
     *            Whether or not should the output sign should be reversed.
     */
    public void setInversion(boolean reversed)
    {
        this.reversed = reversed ? -1.0 : 1.0;
    }
}
