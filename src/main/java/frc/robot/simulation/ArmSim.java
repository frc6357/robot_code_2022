package frc.robot.simulation;

/**
 * A Class to simulate the arm motions
 */
public class ArmSim
{

    /**
     * Base length (non-extendable portion of the Arm).
     */
    private final int baseLength;

    /**
     * Maximum Extension length of the arm (extendable portion of the arm - note this is
     * ADDITIVE to the Base Length, not overlapping).
     */
    private final int maxExtendLength;

    /**
     * The minimum angle of the arm, when tilted downward.
     */
    private final int minAngle;

    /**
     * The maximum angle of the arm, when upright.
     */
    private final int maxAngle;

    /**
     * The currently extended length of the arm (will be between zero and the
     * maxExtendLength).
     */
    private double extendLength = 0;

    /**
     * The current tilt angle of the arm.
     */
    private double degreeLocation = 90;

    /**
     * Constructs a new ArmSim.
     * 
     * @param baseLength
     *            The base arm length for the arm in this ArmSim
     * @param maxExtendLength
     *            The maximum extend length for the arm in this ArmSim
     * @param minAngle
     *            The minimum angle for the arm in this ArmSim
     * @param maxAngle
     *            The maximum angle for the arm in this ArmSim
     */
    public ArmSim(int baseLength, int maxExtendLength, int minAngle, int maxAngle)
    {
        this.baseLength = baseLength;
        this.maxExtendLength = maxExtendLength;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    /**
     * Sets the current extend length of the arm.
     * 
     * @param length
     *            the length to which the extension is set
     */
    public void setExtendLength(double length)
    {
        if (length < 0)
        {
            extendLength = 0;
            return;
        }
        if (length > maxExtendLength)
        {
            extendLength = maxExtendLength;
            return;
        }
        extendLength = length;
    }

    /**
     * Changes the extend length of the arm by the given delta value (can be positive or
     * negative).
     * 
     * @param delta
     *            The delta to be added to the current extendLength
     */
    public void changeExtendLength(double delta)
    {
        setExtendLength(extendLength + delta);
    }

    /**
     * Gets the current extend length of the arm.
     * 
     * @return The current extend length of the arm
     */
    public double getExtendLength()
    {
        return extendLength;
    }

    public int getBaseLength()
    {
        return baseLength;
    }

    public double getTotalLength()
    {
        return extendLength + baseLength;
    }

    public void changeAngle(double deltaDegrees)
    {
        setAngle(degreeLocation + deltaDegrees);
    }

    private void setAngle(double newDegreeLocation)
    {
        if (newDegreeLocation < minAngle)
        {
            degreeLocation = minAngle;
        }
        else if (newDegreeLocation > maxAngle)
        {
            degreeLocation = maxAngle;
        }
        else
        {
            degreeLocation = newDegreeLocation;
        }
    }

    public double getAngle()
    {
        return degreeLocation;
    }

}
