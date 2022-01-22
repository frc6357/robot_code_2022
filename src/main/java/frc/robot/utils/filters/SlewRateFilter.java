package frc.robot.utils.filters;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class SlewRateFilter implements Filter
{

    private final SlewRateLimiter slewFilter;

    public SlewRateFilter(double filterRate)
    {
        slewFilter = new SlewRateLimiter(filterRate);
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
        return slewFilter.calculate(rawAxis);
    }
}
